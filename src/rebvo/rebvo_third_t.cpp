/******************************************************************************

   REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
   Copyright (C) 2016  Juan José Tarrio

   Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
   for a Monocular Camera. In Proceedings of the IEEE International Conference
   on Computer Vision (pp. 702-710).

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

 *******************************************************************************/



#include <TooN/so3.h>
#include <iostream>
#include <iomanip>
#include <TooN/Cholesky.h>


#include "rebvo/rebvo.h"
#include "UtilLib/ttimer.h"
#include "VideoLib/datasetcam.h"
#include "mtracklib/scaleestimator.h"

#include "VideoLib/video_mfc.h"
#include "VideoLib/video_mjpeg.h"
#include "CommLib/net_keypoint.h"
#include "CommLib/udp_port.h"



using namespace std;

namespace  rebvo{

void REBVO::ThirdThread(REBVO *cf){
    using namespace TooN;

    bool quit=false;

    /*****   Set cpu Afinity of the thread   ******/

    if(cf->params.cpuSetAffinity){
        if(!REBVO::setAffinity(cf->params.cpu2)){
            std::cout <<"REBVO: Cannot set cpu affinity on the third thread";
            cf->quit=true;
        }
    }



    /***** Init a pipe to buffer the NET packets *****
            This is nedded to compesate for the delay
            introduced by the MFC encoder, if used,
            and can be configured with the EdgeMapDelay
            tag */


    const int flen=MFC_MAX_STREAM_SIZE+sizeof(net_packet_hdr)+sizeof(net_keyline)*KEYLINE_MAX;
    int hdr_payload=0;


    util::CircListIndexer net_buf_inx(cf->params.EdgeMapDelay+1);
    unsigned char **net_pak= new unsigned char*[net_buf_inx.Size()];

    for(int i=0;i<net_buf_inx.Size();i++){
        net_pak[i]=new unsigned char[flen];
        memset(net_pak[i],0,sizeof(net_packet_hdr));
    }

    net_packet_hdr* net_hdr=(net_packet_hdr*)net_pak [0];


    /***** Start UDP comm port if needed******/

    udp_port *com_port=NULL;
    if(cf->params.VideoNetEnabled>0){
        com_port=new udp_port(cf->params.VideoNetHost.data(),cf->params.VideoNetPort);

        if(com_port->Error()){
            cf->quit=true;
            return;
        }
        printf("\nCamaraFrontal: Socket iniciado en %s:%d\n",cf->params.VideoNetHost.data(),cf->params.VideoNetPort);

        com_port->setBlock(cf->params.BlockingUDP>0);

    }

    /****** Init buffer for pla video save (no container) if needed ******/

    int VideoSaveIndex=0,VideoSaveBuffersize=0;
    char * VideoSaveBuffer=NULL;

    if(cf->params.VideoSave>0){
        VideoSaveBuffer=new char[cf->params.VideoSaveBuffersize];
        VideoSaveBuffersize=cf->params.VideoSaveBuffersize;
    }


    /****** Init Encoder if needed ******/

    const Size2D ImageSize(cf->cam.sz);

    VideoEncoder *encoder=NULL;

    if(cf->params.VideoNetEnabled>0 || cf->params.VideoSave>0){
        switch(cf->params.encoder_type){
        case VIDEO_ENCODER_TYPE_MJPEG:
            encoder=new MJPEGEncoder(ImageSize,90);
            break;
        case VIDEO_ENCODER_TYPE_MFC:
        {
            EncoderMFC *encoder_mfc=new EncoderMFC(cf->params.encoder_dev.data());
            int stb_sz=MFC_MAX_STREAM_SIZE;
            if(encoder_mfc->Initialize(ImageSize.w,ImageSize.h,V4L2_PIX_FMT_MPEG4,15,128000,2,2,stb_sz)<0){
                cf->quit=true;
                return;
            }
            encoder_mfc->Autoconfig();
            encoder=encoder_mfc;
        }
            break;
        case VIDEO_ENCODER_TYPE_RAW:
        default:

            encoder=new VideoEncoder(ImageSize);
            break;

        }
    }


    /****** Init log file if needed ******/

    ostringstream a_log,t_log;
    int a_log_inx=0;

    a_log<< std::scientific<<std::setprecision(16);



/*
    ofstream h_log;
    h_log.open("histo_log.txt");
    const uint h_num=500;
    uint histo[h_num];*/

    util::timer t_proc;
    double t_proc_last=0;

    /****** Main Loop does three optional things:
     * Encodes video and sends over UDP
     * Saves encoded video
     * Saves LOG in .m format
     * *******************************************/



    while(!cf->quit && !quit){

        PipeBuffer pbuf=cf->pipe.RequestBuffer(3);          //Request buffer for player 3

        if(pbuf.quit){
            cf->pipe.ReleaseBuffer(3);
            break;
        }

        t_proc.start();


        if(cf->params.VideoNetEnabled>0){

            //****** Prepare network buffer header *****/


            net_hdr= (net_packet_hdr*)net_pak [net_buf_inx-1];
            net_hdr->key_num=0;

            net_hdr->kline_num=copy_net_keyline(*pbuf.ef,cf->params.StereoAvaiable?pbuf.ef_pair:nullptr,\
                                    (net_keyline*)&net_pak [net_buf_inx-1][sizeof(net_packet_hdr)],KEYLINE_MAX,pbuf.K);


            copy_net_keyline_nextid(*pbuf.ef,\
                                    (net_keyline*)&net_pak [net_buf_inx-1][sizeof(net_packet_hdr)],KEYLINE_MAX);


            net_hdr->km_num=pbuf.ef->NumMatches();

            net_hdr->k=pbuf.K;
            net_hdr->max_rho=RHO_MAX;

            net_hdr= (net_packet_hdr*)net_pak [net_buf_inx];

            net_hdr->w=ImageSize.w;
            net_hdr->h=ImageSize.h;
            net_hdr->encoder_type=encoder->GetEncoderType();


            net_hdr->nav.dt=pbuf.dt;
            net_hdr->nav.Pos=pbuf.nav.Pos;
            net_hdr->nav.Pose=pbuf.nav.PoseLie;
            net_hdr->nav.Vel=pbuf.nav.Vel;
            net_hdr->nav.Rot=pbuf.nav.RotLie;


            hdr_payload=sizeof(net_packet_hdr)+sizeof(net_keyline)*net_hdr->kline_num;

            //****** Push frame on encoder *****/

            encoder->PushFrame(pbuf.imgc->Data());

            //****** Pop encoder frame *****/

            int n=encoder->PopFrame((char *)&net_pak[net_buf_inx][hdr_payload]\
                                    ,flen-hdr_payload);


            if(n>=0){                                   //If encoding OK transmit

                net_hdr->data_size=hdr_payload+n;
                net_hdr->jpeg_size=n;

                if (!com_port->SendFragmented(net_pak[net_buf_inx],net_hdr->data_size,32e3)){
                    printf("\nMTrack: Error enviando paquete de %d bytes a %s:%d! %d: %s\n",net_hdr->data_size,cf->params.VideoNetHost.data(),cf->params.VideoNetPort\
                           ,errno, strerror(errno));
                }

                if(cf->params.VideoSave){                      //Optionaly save on file buffer
                    memcpy(&VideoSaveBuffer[VideoSaveIndex],&net_pak[net_buf_inx][hdr_payload],std::min(n,VideoSaveBuffersize-VideoSaveIndex));
                }
            }

            ++net_buf_inx;


        }else if(cf->params.VideoSave && VideoSaveBuffer!=NULL && VideoSaveBuffersize>VideoSaveIndex){

            //****** If only video file buffer enabled encode and decode *****//

            encoder->PushFrame(pbuf.imgc->Data());
            int n=encoder->PopFrame(&VideoSaveBuffer[VideoSaveIndex],VideoSaveBuffersize-VideoSaveIndex);
            VideoSaveIndex+=n;

        }

        if(cf->params.SaveLog){

            //******* Save log data directly on file ******//

            a_log_inx++;

            a_log<<"Kp_cv("<<a_log_inx<<",:)="<<pbuf.Kp<<";\n";
            a_log<<"RKp_cv("<<a_log_inx<<",:)="<<pbuf.RKp<<";\n";
            a_log<<"Rot_cv("<<a_log_inx<<",:,:)=["<<pbuf.nav.Rot(0,0)<<","<<pbuf.nav.Rot(0,1)<<","<<pbuf.nav.Rot(0,2)<<";"\
                <<pbuf.nav.Rot(1,0)<<","<<pbuf.nav.Rot(1,1)<<","<<pbuf.nav.Rot(1,2)<<";"\
               <<pbuf.nav.Rot(2,0)<<","<<pbuf.nav.Rot(2,1)<<","<<pbuf.nav.Rot(2,2)<<"];\n";
            a_log<<"Vel_cv("<<a_log_inx<<",:)=["<<pbuf.nav.Vel[0]<<","<<pbuf.nav.Vel[1]<<","<<pbuf.nav.Vel[2]<<"];\n";
            a_log<<"RotGiro_cv("<<a_log_inx<<",:)=["<<pbuf.nav.RotGiro[0]<<","<<pbuf.nav.RotGiro[1]<<","<<pbuf.nav.RotGiro[2]<<"];\n";
            a_log<<"t_cv("<<a_log_inx<<",:)="<<pbuf.t<<";\n";
            a_log<<"dt_cv("<<a_log_inx<<",:)="<<pbuf.dt<<";\n";
            a_log<<"i_cv("<<a_log_inx<<",:)="<<pbuf.p_id<<";\n";
            a_log<<"Pose_cv("<<a_log_inx<<",:,:)=["<<pbuf.nav.Pose(0,0)<<","<<pbuf.nav.Pose(0,1)<<","<<pbuf.nav.Pose(0,2)<<";"\
                <<pbuf.nav.Pose(1,0)<<","<<pbuf.nav.Pose(1,1)<<","<<pbuf.nav.Pose(1,2)<<";"\
               <<pbuf.nav.Pose(2,0)<<","<<pbuf.nav.Pose(2,1)<<","<<pbuf.nav.Pose(2,2)<<"];\n";
            a_log<<"Pos_cv("<<a_log_inx<<",:)=["<<pbuf.nav.Pos[0]<<","<<pbuf.nav.Pos[1]<<","<<pbuf.nav.Pos[2]<<"];\n";
            a_log<<"K_cv("<<a_log_inx<<",:)="<<pbuf.K<<";\n";
            a_log<<"KLN_cv("<<a_log_inx<<",:)="<<pbuf.ef->KNum()<<";\n";


            a_log<<"Giro_cv("<<a_log_inx<<",:)=["<<pbuf.imu.giro[0]<<","<<pbuf.imu.giro[1]<<","<<pbuf.imu.giro[2]<<"];\n";
            a_log<<"Acel_cv("<<a_log_inx<<",:)=["<<pbuf.imu.acel[0]<<","<<pbuf.imu.acel[1]<<","<<pbuf.imu.acel[2]<<"];\n";
            a_log<<"CAcel_cv("<<a_log_inx<<",:)=["<<pbuf.imu.cacel[0]<<","<<pbuf.imu.cacel[1]<<","<<pbuf.imu.cacel[2]<<"];\n";
            a_log<<"DGiro_cv("<<a_log_inx<<",:)=["<<pbuf.imu.dgiro[0]<<","<<pbuf.imu.dgiro[1]<<","<<pbuf.imu.dgiro[2]<<"];\n";

            a_log<<"GBias_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Bg[0]<<","<<pbuf.imustate.Bg[1]<<","<<pbuf.imustate.Bg[2]<<"];\n";
            a_log<<"dWv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.dWv[0]<<","<<pbuf.imustate.dWv[1]<<","<<pbuf.imustate.dWv[2]<<"];\n";
            a_log<<"dWgv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.dWgv[0]<<","<<pbuf.imustate.dWgv[1]<<","<<pbuf.imustate.dWgv[2]<<"];\n";


            a_log<<"g_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.g_est[0]<<","<<pbuf.imustate.g_est[1]<<","<<pbuf.imustate.g_est[2]<<"];\n";
            a_log<<"VBias_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.b_est[0]<<","<<pbuf.imustate.b_est[1]<<","<<pbuf.imustate.b_est[2]<<"];\n";
            a_log<<"Av_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Av[0]<<","<<pbuf.imustate.Av[1]<<","<<pbuf.imustate.Av[2]<<"];\n";
            a_log<<"As_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.As[0]<<","<<pbuf.imustate.As[1]<<","<<pbuf.imustate.As[2]<<"];\n";

            a_log<<"Posgv_cv("<<a_log_inx<<",:)=["<<pbuf.imustate.Posgv[0]<<","<<pbuf.imustate.Posgv[1]<<","<<pbuf.imustate.Posgv[2]<<"];\n";


            a_log<<"SMM_cv("<<a_log_inx<<",:)="<<pbuf.stereo_match_num<<";\n";

            a_log<<"TProc0_cv("<<a_log_inx<<",:)="<<pbuf.dtp0<<";\n";
            a_log<<"TProc1_cv("<<a_log_inx<<",:)="<<pbuf.dtp1<<";\n";
            a_log<<"TProc2_cv("<<a_log_inx<<",:)="<<t_proc_last<<";\n";

            //a_log.flush();

            //******* Save trayectory ************//
            TooN::Vector<4> q = util::LieRot2Quaternion(pbuf.nav.PoseLie);
            t_log << std::scientific<<std::setprecision(18)<< pbuf.t << " " <<pbuf.nav.Pos << q[0] << " " << q[1] << " " \
            << q[2] << " " << q[3] << "\n";
            //t_log.flush();

        }


        COND_TIME_DEBUG(printf("\nCamara TT Dtp=%f\n",t_proc_last);)
/*

        pbuf.ef->DebugMatchHisto(500,h_num,histo);
        for(int i=0;i<h_num-1;i++)
            h_log<<histo[i]<<",";
        h_log<<histo[h_num-1]<<"\n";
        h_log.flush();*/


        //******** Call callback if present *****************//

        cf->callCallBack(pbuf);




          t_proc_last=t_proc.stop();
        //******** The system can optionaly take a snapshot (use for debuging purposes) ******//

        if(cf->saveImg){
            cf->saveImg=false;
            char name[128];
            snprintf(name,sizeof(name),"Snap%d.ppm",cf->snap_n);
            cout << "\nCamara Frontal: Tomando foto"<<cf->snap_n++<<"\n";
            SavePPM(name,pbuf.imgc->Data(),ImageSize.w,ImageSize.h);
        }

        cf->pipe.ReleaseBuffer(3);

    }

    // ****** All the video is saved in RAM and saved at the end, for avoiding delays ******//

    if(cf->params.VideoSave){

        printf("\nCamara Frontal: Saving video file...\n");

        ofstream video_of;
        video_of.open(cf->params.VideoSaveFile.data(),ios_base::trunc);

        if(video_of.is_open()){

            video_of.write(VideoSaveBuffer,VideoSaveIndex);
            video_of.close();

        }else{
            printf("\nCamara Frontal: Failed to open video save file\n");
        }
    }

    if(encoder)
        delete encoder;

    ofstream fa_log,ft_log;

    if(cf->params.SaveLog){

        fa_log.open(cf->params.LogFile.data());
        if(!fa_log.is_open()){
            cout <<"\nREBVO: Cannot open log file\n";
            cf->quit=true;
            return;
        }else{


            cout <<"\nREBVO: Saving log file\n";
            fa_log.write(a_log.str().data(),a_log.str().size());

            fa_log.close();

        }

        ft_log.open(cf->params.TrayFile.data());
        if(!ft_log.is_open()){
            cout <<"\nREBVO: Cannot open trayectory file\n";
            cf->quit=true;
            return;
        }else{

            ft_log.write(t_log.str().data(),t_log.str().size());
            ft_log.close();
        }

    }

    //h_log.close();

    cf->quit=true;

    return;


}

}
