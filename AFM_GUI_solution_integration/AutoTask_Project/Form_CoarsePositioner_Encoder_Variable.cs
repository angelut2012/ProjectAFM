using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Threading;
using System.IO.Ports;
using System.Text.RegularExpressions;
using System.Diagnostics;
using System.Runtime.InteropServices;
using TDx.TDxInput;
// start other program

namespace NameSpace_AFM_Project
{
    public partial class MainWindow : Form
    {
        bool caliMatAppBool;
        bool caliSPBool;
        bool InitStepPre;
        bool m_bConnexion;
        bool m_bCTLConnected;
        bool m_bCVTConnected;
        bool m_bCVTConnected_init;
        bool m_bCXConnected;
        bool m_bEnReader;
        bool m_bGPedC;
        bool m_bGPedO;
        bool m_bGripper;
        bool m_bJSKConnected;
        bool m_bJSKOn;
        bool m_bXDOnly;
        bool m_bYDOnly;
        bool m_bZCompensated;
        bool m_bZDOnly;
        bool PlungerSts1;
        bool PlungerSts2;
        bool posReplayDir;
        bool posReplayRecordBool;
        bool SCCStatusInit;
        bool SCCStatusRecv;
        bool SCCVRB;
        bool tickOutXY;
        bool tickOutZ;
        bool[] posHeightControlBool = new bool[NUMMANIVT];
        bool[] posReferenceZBool = new bool[NUMMANIVT];
        char dSEMHLMag;
        char dSEMRRotOn;
        char dSEMScanMode;
        char dSEMScanSpd;
        double _DFBK_X;
        double _DFBK_Y;
        double AutomatePos_sts;
        double Byte_CTL;
        double db_PlungerSts1;
        double db_PlungerSts2;
        double dSEMCur;
        double dSEMCurPre;
        double dSEMMag_base;
        double dSEMMag_base_pre;
        double dSEMMag_expo;
        double dSEMMag_expo_Mod;
        double dSEMMag_expo_pre;
        double dSEMR;
        double dSEMRRotData;
        double dSEMT;
        double dSEMVol;
        double dSEMVolPre;
        double dSEMX;
        double dSEMY;
        double dSEMZ;
        double dwTickAfter;
        double dwTickAfter_CTL;
        double dwTickBefore;
        double dwTickBefore_CTL;
        double eExcPos;
        double eExcPosPre;
        double eHomePos;
        double eHomePosPre;
        double GripperArm;
        double GripperLSts1;
        double GripperLSts2;
        double GripperRSts1;
        double GripperRSts2;
        double jCF;
        double jFMSSelc;
        double jGripperAction;
        double jGripperPower;
        double jGuardShield;
        double jLBRSelc;
        double jLFCarrier;
        double jManiSelc;
        double jPlungerPower;
        double jPlungerPowerPre;
        double jPlungerSpd;
        double jPlungerSpdPre;
        double jPositionerSelc;
        double jSSSelc;
        double jSSStatusXY;
        double jSSStatusXY_Fine;
        double jSSStatusZ;
        double jSSStatusZ_Fine;
        double jsValueX;
        double jsValueY;
        double jXMove;
        double jXStep;
        double jXStepPre;
        double jYMove;
        double jYStep;
        double jYStepPre;
        double jZActionEmergency;
        double jZActionEmergencyCount;
        double jZStep;
        double jZStepPre;
        double LF_Bias1;
        double LF_Bias2;
        double m_bCFVT;
        double m_blnkMagClosedStopped;
        int ManiSelcVT;
        double MoveDirec;
        double PlungerSpd;
        double ReadoutCount;
        double SCCStatusCount;
        double SCCStatusTimeOut;
        double SCCVRRecv;
        int SETVT;
        double test_axis;
        double tickCRX;
        double tickCRY;
        double tickThresXY;
        double tickThresZ;
        double tickXY;
        double tickZ;
        double ulBytesReadRequest;
        double ulBytesReadRequest_CTL;
        double ulBytesSucceed;
        double ulBytesSucceed_CTL;
        double ulBytesWriteRequest;
        double ulBytesWriteRequest_CTL;
        double valueRX;
        double valueRZ;
        double valueTY;
        double VRRFVT;
        double XPosFeedBk;
        double YPosFeedBk;
        double zDisDown;
        double zNeg;
        double zPos;
        double ZPosFeedBk;
        double zStepNeg;
        double zStepPos;
        double ZStepSizeIndexCVT;
        double[, , ,] PIDVT = new double[NUMMANIVT, 3, 3, 2];
        double[, ,] boundVT = new double[NUMMANIVT, 3, 2];
        double[, ,] CaliMatPosSec = new double[NUMMANIVT, 3, 3];
        double[, ,] SCCVR = new double[NUMMANIVT, 3, 2];
        double[, ,] volRangeVT = new double[NUMMANIVT, 3, 2];
        double[, ,] VRFVT = new double[NUMMANIVT, 3, 2];
        double[,] boundDisVT = new double[NUMMANIVT, 3];
        double[,] degreeVT = new double[NUMMANIVT, 3];
        double[,] ledsVT = new double[NUMMANIVT, 6];
        double[,] noCR = new double[NUMMANIVT, 3];
        double[,] noVT = new double[NUMMANIVT, 3];
        double[,] orientCR = new double[NUMMANIVT, 3];
        double[,] orientVT = new double[NUMMANIVT, 3];
        double[,] posEncoderVT = new double[NUMMANIVT, 3];
        double[,] posXYZ_vol = new double[NUMMANIVT, 3];
        double[,] slopeVT = new double[NUMMANIVT, 3];
        double[] io_buffer = new double[64];
        double[] io_buffer_CTL = new double[4];
        double[] io_buffer_CTL_r = new double[64];
        double[] leds = new double[16];
        double[] ledsAuto = new double[6];
        double[] ledsCali = new double[6];
        double[] m_iTimerIntl = new double[5];
        double[] MnlCaliOffSet = new double[NUM_CH_SMARACT];
        double[] NP_Index = new double[NPNUM * NPAXISNUM];
        double[] posHeightControlValue = new double[NUMMANIVT];
        double[] PositionerStsNP = new double[NUM_CH_SMARACT];
        double[] PositionerStsNP_INIT = new double[NUM_CH_SMARACT];
        double[] PositionerStsNP_MAX = new double[NUM_CH_SMARACT];
        double[] PositionerStsNP_MIN = new double[NUM_CH_SMARACT];
        double[] posReferenceZ = new double[NUMMANIVT];
        double[] posReferenceZData = new double[NUMMANIVT];
        double[] sbuffer = new double[256];
        double[] ZStepSizeIndexVT = new double[NUMMANIVT];
        string sMessage;
    };
}