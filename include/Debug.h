
//----------------------------------------------------------//
//                                                          //
//                        赛前调试参数                       //
//                                                          //
//----------------------------------------------------------//
#define DEBUG_AUTOAIM
#define ARMOR_RED    // 检测红方装甲板,识别蓝色能量机关

// #define DEBUG_BUFF
// #define ARMOR_BLUE   // 检测蓝方装甲板，识别红色能量机关

#ifdef ARMOR_RED
// #define E_BLUE       // 识别蓝色能量机关
#endif // A_RED

#ifdef ARMOR_BLUE
// #define E_RED   // 识别红色能量机关
#endif // A_BLUE

//----------------------------------------------------------//
//                                                          //
//                         线程调试参数                       //
//                                                          //
//----------------------------------------------------------//
// #define MULTI_THREAD     // 是否使用多线程

// #define COMPILE_WITH_GPU       // FIXME:辅瞄是否使用GPU(不适用于妙算)

#define DEBUG_WITHOUT_COM         //无串口调试

// #define SAVE_VIDEO_DAHENG      //是否录制视频并保存到本地(大恒)
// #define SAVE_VIDEO_USB_CAMERA  //是否录制视频并保存到本地(USB相机).

//----------------------------------------------------------//
//                                                          //
//                        图像调试参数                        //
//                                                          //
//----------------------------------------------------------//
// #define USE_LOCAL_VIDEO
#define SHOW_FRAME

//-----------------------------(辅瞄)-----------------------------------//
// #define USE_LOCAL_VIDEO_ARMOR   // 是否使用本地测试视频
// #define USE_DAHENG_CAMERA // 是否使用大恒139相机
#define USE_USB_CAMERA    // 是否使用USB相机

// #define REOPEN            //是否在丢帧后重启摄像头

// #define SHOW_SRC          // 是否显示原始图像
#define SHOW_GYRING
#define SHOW_DISTANCE
// #define SHOW_DEBUG_IMG
//-----------------------------(大符)-----------------------------------//
// #define USE_LOCAL_VIDEO_BUFF          // 是否使用本地测试视频ss
// #define USE_USB_CAMERA_ENERGY // 是否使用USB相机
// #define USE_LOCAL_VIDEO_ENERGY   // 是否使用本地测试视频

//----------------------------------------------------------//
//                                                          //
//                                                          //
//                      终端输出参数                     //
//                                                          //
//----------------------------------------------------------//
// #define SHOW_PREDICT        // 是否显示陀螺状态预测参数（自瞄）
// #define SHOW_SRC_GET_TIME // 是否在终端打印每帧的采集时间(仅限于大恒)
// #define SHOW_ENERGY_RUN_TIME // 是否在终端打印能量机关每帧的处理时间
// #define CALC_PROCESS_TIME    // 显示生产及消费所用时间(辅瞄)
#define ECHO_FINAL_INFO      // 是否输出最终得到的Yaw,Pitch,Dist信息          
// #define COUT_LOG             //输出日志

//----------------------------------------------------------//
//                                                          //
//                                                          //
//                      功能选择参数                          //
//                                                          //
//----------------------------------------------------------//
// #define USING_ADVANCED_PREDICT    //是否为辅瞄启用进阶预测(卡尔曼与反陀螺)(未完成)
// #define USING_DEBUG_ANTISPIN  //反陀螺(未完成)
// #define ENABLE_NUM_CLASSFICATION  //启用SVM
#define svm

//--------------------//
//                    //
//    串口类功能调试    //
//                    //
//--------------------//
// #define GET_MODE
// #define GET_DATA


