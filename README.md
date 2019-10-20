# CNNIF
CNN-based fractional interpolation filter for video coding on top of HM16.7.
C++ interface of Caffe should be setted up to run this code.

## Macro explanation
- CNN_FRAC_INTERPOLATION : enable CNN-based interpolation filter
- ENABLE_CU_LEVEL_RDO : enable CU-level filter type decision
- ENABLE_BI_MODEL : enable separate interpolation filters for list0 and list1

## Question and answer
If you have any questiones, please do not hesitate to contact us through nyan@mail.ustc.edu.cn

## License and Citation
Please cite our work in your publications if it helps your research:

  @inproceedings{yan2017convolutional,
      title={A convolutional neural network approach for half-pel interpolation in video coding},
      author={Yan, Ning and Liu, Dong and Li, Houqiang and Wu, Feng},
      booktitle={Circuits and Systems (ISCAS), 2017 IEEE International Symposium on},
      pages={1--4},
      year={2017},
      organization={IEEE}
  }

  @article{yan2018convolutional,
      title={Convolutional Neural Network-Based Fractional-Pixel Motion Compensation},
      author={Yan, Ning and Liu, Dong and Li, Houqiang and Li, Bin and Li, Li and Wu, Feng},
      journal={IEEE Transactions on Circuits and Systems for Video Technology},
      year={2018},
      publisher={IEEE}
  }

  @article{yan2019invif,
      title={Invertibility-Driven Interpolation Filter for Video Coding},
      author={Yan, Ning and Liu, Dong and Li, Houqiang and Li, Bin and Li, Li and Wu, Feng},
      journal={IEEE Transactions on Image Processing},
      year={2019},
      publisher={IEEE}
  }
