#ifndef __TCOMCONVNET__
#define   __TCOMCONVNET__

#include "TComCaffe.h"

class TComConvNet
{
private:
  string                    m_netFileName;
  string                    m_caffeModelName;
  TComCaffe                 m_pcCaffe;
  cv::Mat                   m_inputBlock;
  std::vector<Float>        m_networkOutput;
  Int                       m_iInputHeight;
  Int                       m_iInputWidth;

public:
  TComConvNet();
  ~TComConvNet();

  Void                creat(const string& netFile, const string& modelFile, TChar* query_blob_name, TChar* mode, Int bitDepth);
  Void                destroy(const string& netFile, const string& modelFile, TChar* mode);

  Int                 getInputHeight(){ return m_iInputHeight; }
  Int                 getInputWidth() { return m_iInputWidth;  }

  Void                setInutBlkSize( Int inputHeight, Int inputWidth ); // set input size
  Void                setInputBlock ( const Pel* src, Int Stride, cv::Mat& inputMat, Int inputHeight, Int inputWidth );
  cv::Mat             getInputBlock ( ) { return m_inputBlock; }

  Void                ConvNetForward(cv::Mat& inputMat);
  std::vector<Float>  getConvNetOutput();
  std::vector<Float>  getConvNetOutputCenter(Int iMargin);

#if CNN_FRAC_INTERPOLATION
  Void copyFromVectorToPointer(std::vector<Float>& src, Pel* pDst, Int iStrideDst, Int iheight, Int iwidth, Int bitDepth, Bool isLast = true);
  // get net file and caffe model
  string getCNNIFNetFile(Int yFrac, Int xFrac, Int iQP);
  string getCNNIFCaffeModel(Int yFrac, Int xFrac, Int iQP, RefPicList eRefList);
#endif
};
#endif