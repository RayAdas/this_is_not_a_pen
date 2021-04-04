#include "galaxycamera.h"

galaxycamera::galaxycamera(){

}
bool galaxycamera::cameraInit(){
    status=GX_STATUS_SUCCESS;
    nDeviceNum=0;
    status = GXInitLib();
    hDevice = NULL;
    if (status != GX_STATUS_SUCCESS){
        //1.initial ku
        std::cout<<"库initial error!"<<std::endl;
        return 0;
    }
    //2.list&open camera
    status=GXUpdateDeviceList(&nDeviceNum,1000);
    if((status!=GX_STATUS_SUCCESS)||(nDeviceNum<=0)){
        std::cout<<"camera list error"<<std::endl;
        return 0;
    }
    status = GXOpenDeviceByIndex(1, &hDevice);//3.open device
    status=GXSetFloat(hDevice,GX_FLOAT_EXPOSURE_TIME,5500.0);//4.setting configurations
    status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_ONCE);//
    if (status == GX_STATUS_SUCCESS){
        //开 采
        status = GXStreamOn(hDevice);
    }
    return 1;
}
bool galaxycamera::IGetFrame(Mat &src){
    if (status == GX_STATUS_SUCCESS){
    //调 用 GXDQBuf 取 一 帧 图 像
        status = GXDQBuf(hDevice, &pFrameBuffer, 1000);
        if (status == GX_STATUS_SUCCESS){
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS){
                 //图 像 获 取 成 功
                //对 图 像 进 行 处 理 ...
                int64_t *pRGB24Buf=new(std::nothrow)int64_t[nWidth * nHeight * 3]; //输 出 图 像 RGB 数 据
                if (pRGB24Buf == NULL){
                    return 0;
                }
                DX_BAYER_CONVERT_TYPE cvtype=RAW2RGB_NEIGHBOUR; //选 择 插 值 算 法
                DX_PIXEL_COLOR_FILTER nBayerType = DX_PIXEL_COLOR_FILTER(GX_COLOR_FILTER_BAYER_BG);//BGR MODE
                bool bFlip = true;
                VxInt32 DxStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf,pRGB24Buf,nWidth,nHeight,cvtype,nBayerType,bFlip);
                //BAYER CHANGE BGR MODE
                if (DxStatus != DX_OK){
                    if (pRGB24Buf != NULL){
                        delete []pRGB24Buf;
                        pRGB24Buf = NULL;
                        }
                    return 0;
                }
                //对 BGR24 数 据 进 行 处 理...........................
                Mat img=cv::Mat(nHeight,nWidth,CV_8UC3,pRGB24Buf);
                Size a(640,512);
                cv::resize(img,src,a);
                cv::flip(src,src,1);
                if (pRGB24Buf != NULL){
                    delete []pRGB24Buf;
                    pRGB24Buf = NULL;
                    }
                }
            //调 用 GXQBuf 将 图 像 buf 放 回 库 中 继 续 采 图
            status = GXQBuf(hDevice, pFrameBuffer);
            }
        }
}
