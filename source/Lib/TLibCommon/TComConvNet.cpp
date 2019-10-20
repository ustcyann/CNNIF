
#include "TComCaffeHeader.h"
#include "TComConvNet.h"
#include "TComCaffe.h"

#include "cassert"
#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "TComPicYuv.h"
#include "TComYuv.h"
#include "TComMv.h"
#include "TComDataCU.h"
#include "TComPic.h"
#include "TComInterpolationFilter.h"

using std::string;

TComConvNet::TComConvNet()
{

}

TComConvNet::~TComConvNet()
{

}

Void TComConvNet::creat(const string& netFile, const string& modelFile, TChar* query_blob_name, TChar* mode, Int bitDepth)
{
  m_pcCaffe.creat(netFile, modelFile, query_blob_name, mode, bitDepth);
}

Void TComConvNet::destroy(const string& netFile, const string& modelFile, TChar* mode)
{
  m_pcCaffe.destroy(netFile, modelFile, mode);
}

Void TComConvNet::setInutBlkSize(Int inputHeight, Int inputWidth)
{
  m_pcCaffe.setInputSize(cv::Size(inputWidth, inputHeight));
  m_iInputHeight = inputHeight;
  m_iInputWidth  = inputWidth;
}

Void TComConvNet::setInputBlock(const Pel* src, Int Stride, cv::Mat& inputMat, Int inputHeight, Int inputWidth)
{
  //m_pcCaffe.setInputSize(cv::Size(inputWidth, inputHeight));
  //m_iInputHeight = inputHeight;
  //m_iInputWidth  = inputWidth;

  for (Int y = 0; y < inputHeight; y++)
  {
    for (Int x = 0; x < inputWidth; x++)
    {
      inputMat.at<Float>(y, x) = (Float)src[x];
    }
    src += Stride;
  }
}

Void TComConvNet::ConvNetForward(cv::Mat& inputMat)
{
  m_pcCaffe.netForward(inputMat);
  m_networkOutput = m_pcCaffe.getNetOutput();
}

std::vector<Float> TComConvNet::getConvNetOutput()
{
  return m_networkOutput;
}

std::vector<Float> TComConvNet::getConvNetOutputCenter(Int iMargin)
{
  Int iheight = getInputHeight();
  Int iwidth  = getInputWidth();
  Int iheightDst = iheight - 2 * iMargin;
  Int iwidthDst  = iwidth  - 2 * iMargin;

  std::vector<Float> netOut = m_networkOutput;
  std::vector<Float>::iterator it = netOut.begin();
  it += (iMargin*iwidth + iMargin);

  vector<Float> dst;
  for (Int y = 0; y < iheightDst; y++)
  {
    for (Int x = 0; x < iwidthDst; x++)
    {
      dst.push_back(it[y*iwidth + x]);
    }
  }
  return dst;
}

#if CNN_FRAC_INTERPOLATION
Void TComConvNet::copyFromVectorToPointer(std::vector<float>& src, Pel* pDst, Int iStrideDst, Int iheight, Int iwidth, Int bitDepth, Bool isLast)
{

  Int offset = (1 << IF_INTERNAL_PREC) - 1;
  Int shift  = IF_INTERNAL_PREC - bitDepth;

  if (isLast)
  {
    for (Int y = 0; y < iheight; y++)
    {
      for (Int x = 0; x < iwidth; x++)
      {
        Pel temp = (Pel)(offset * src[y*iwidth + x] + 0.5); 
        temp = (temp >> shift);
        temp = Clip3<Pel>(0, 255, temp);
        pDst[x] = temp;
      }
      pDst += iStrideDst;
    }
  }
  else
  {
    for (Int y = 0; y < iheight; y++)
    {
      for (Int x = 0; x < iwidth; x++)
      {
        pDst[x] = (Pel)(offset * src[y*iwidth + x] + 0.5); // scale the value to 14 bits
        pDst[x] = pDst[x] - IF_INTERNAL_OFFS;
      }
      pDst += iStrideDst;
    }
  }
}

string TComConvNet::getCNNIFNetFile(Int yFrac, Int xFrac, Int iQP)
{
  string net_file = "./CaffeModel/VRCNN_net.prototxt";
  //net_file = "./CaffeModel/R1_InvIF_3layer/InvIF_3layer.prototxt";

  //net_file = "./CaffeModel/R1_InvIF_6layer/InvIF_6layer.prototxt";
  //net_file = "./CaffeModel/R1_InvIF_8layer/InvIF_8layer.prototxt";

  return net_file;
}

string TComConvNet::getCNNIFCaffeModel(Int yFrac, Int xFrac, Int iQP, RefPicList eRefList)
{
  string caffeModel;
  string caffeModel_Path;
  string caffeModel_Name = "IF_phXX_QXX_iter_200000.caffemodel";

#if ONLY_USE_QP22_MODEL
  iQP = 22;
#endif

  Int qp[] = { 22, 27, 32, 37 };
  Int iBestIdx = 0;
  Int iBestDiff = abs(iQP - 22);
  for (Int i = 0; i < 4; i++)
  {
    Int iDiff = abs(iQP - qp[i]);
    if (iDiff < iBestDiff)
    {
      iBestDiff = iDiff;
      iBestIdx = i;
    }
  }
  iQP = qp[iBestIdx];

  TChar QPStr[] = { iQP / 10 + 48, iQP % 10 + 48, '\0' };
  TChar cPhase[] = { '0', '1', '2', '3' };

  Int Pos1 = 9;
  Int Pos2 = 10;
  Int yFracPos = 5;
  Int xFracPos = 6;

  caffeModel_Name[yFracPos] = cPhase[yFrac];
  caffeModel_Name[xFracPos] = cPhase[xFrac];
  caffeModel_Name[Pos1] = QPStr[0];
  caffeModel_Name[Pos2] = QPStr[1];

  if (eRefList == REF_PIC_LIST_0)
  {
    caffeModel_Path = "./CaffeModel/CNNIF/";

    //caffeModel_Path = "./CaffeModel/IF_VRCNN_Train_BlowingBubbles_HAndQ/";

    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w0/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w02/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w04/";
    caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w05/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w06/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w08/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualTrain_Ex2_2_Flip_BlockVar_th6_mid_w1/";

    //caffeModel_Path = "./CaffeModel/R1_InvIF_6layer/";
    //caffeModel_Path = "./CaffeModel/R1_InvIF_3layer/";
    //caffeModel_Path = "./CaffeModel/R1_InvIF_8layer/";



    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualIF_Ex2_2_BicubicReg_w05/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/IF_VRCNN_DualIF_Ex2_2_BilinearReg_w05/";

    //caffeModel_Path = "./CaffeModel/DualIF_Ex2_2/ISCAS_CNNIF/";

    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w02/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w04/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w05/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w06/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w07/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w08/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w09/";
    //caffeModel_Path = "./CaffeModel/DualIF_Ex1_2/IF_VRCNN_DualTrain_Ex1_2_Flip_BlockVar_th6_mid_w1/";

  }
  else
  {
    caffeModel_Path = "./CaffeModel/IF_VRCNN_TrainBlow_List1_Model/";
  }


  caffeModel = caffeModel_Path + caffeModel_Name;
  return caffeModel;
}
#endif