// elmue_proto.h —— ElmüSoft Candlelight 2.5 协议常量与帧结构
//
// 抽取自 rs00_comm_test.cpp（第 43-256 行），归属上游：
//   elmue/elmue_proto.h — ElmüSoft GitHub 公开实现
//
// 用法:
//   #include "elmue_can/elmue_proto.h"

#ifndef ELMUE_CAN__ELMUE_PROTO_H_
#define ELMUE_CAN__ELMUE_PROTO_H_

#include <cstdint>

#define ELMUE_PACKED __attribute__((packed, aligned(1)))

#define ELMUE_VENDOR_ID  0x1d50
#define ELMUE_PRODUCT_ID 0x606f

#define ELMUE_EP_IN  0x81
#define ELMUE_EP_OUT 0x02

namespace elmue_can {

typedef enum {
    GS_ReqSetHostFormat = 0,
    GS_ReqSetBitTiming,
    GS_ReqSetDeviceMode,
    GS_ReqBerrReport,
    GS_ReqGetCapabilities,
    GS_ReqGetDeviceVersion,
    GS_ReqGetTimestamp,
    GS_ReqIdentify,
    GS_ReqGetUserID,
    GS_ReqSetUserID,
    GS_ReqSetBitTimingFD,
    GS_ReqGetCapabilitiesFD,
    GS_ReqSetTermination,
    GS_ReqGetTermination,
    GS_ReqGetState,

    ELM_ReqFIRST = 20,
    ELM_ReqGetBoardInfo = 20,
    ELM_ReqSetFilter,
    ELM_ReqGetLastError,
    ELM_ReqSetBusLoadReport,
    ELM_ReqSetPinStatus,
    ELM_ReqGetPinStatus,
    ELM_ReqReadFlash,
    ELM_ReqWriteFlash,
} eUsbRequest;

typedef enum {
    GS_DevFlagNone = 0,
    GS_DevFlagListenOnly = 0x00001,
    GS_DevFlagLoopback = 0x00002,
    GS_DevFlagTripleSample = 0x00004,
    GS_DevFlagOneShot = 0x00008,
    GS_DevFlagTimestamp = 0x00010,
    GS_DevFlagIdentify = 0x00020,
    GS_DevFlagUserID = 0x00040,
    GS_DevFlagPadPacketsToMaxSize = 0x00080,
    GS_DevFlagCAN_FD = 0x00100,
    GS_DevFlagQuirk_LPC546XX = 0x00200,
    GS_DevFlagBitTimingFD = 0x00400,
    GS_DevFlagTermination = 0x00800,
    GS_DevFlagBerrReporting = 0x01000,
    GS_DevFlagGetState = 0x02000,

    ELM_DevFlagProtocolElmue = 0x04000,
    ELM_DevFlagDisableTxEcho = 0x08000,
} eDeviceFlags;

typedef enum {
    FBK_Success = 2,
    FBK_InvalidCommand = '1',
    FBK_InvalidParameter,
    FBK_AdapterMustBeOpen,
    FBK_AdapterMustBeClosed,
    FBK_ErrorFromHAL,
    FBK_UnsupportedFeature,
    FBK_TxBufferFull,
    FBK_BusIsOff,
    FBK_NoTxInSilentMode,
    FBK_BaudrateNotSet,
    FBK_OptBytesProgrFailed,
    FBK_ResetRequired,
    FBK_ParamOutOfRange,
} eFeedback;

typedef enum {
    GS_ModeReset = 0,
    GS_ModeStart,
} eDeviceMode;

typedef enum {
    CAN_ID_Error = 0x20000000,
    CAN_ID_RTR = 0x40000000,
    CAN_ID_29Bit = 0x80000000,
    CAN_MASK_11 = 0x000007FF,
    CAN_MASK_29 = 0x1FFFFFFF,
} eCanIdFlags;

typedef enum {
    FRM_Overflow = 0x01,
    FRM_FDF = 0x02,
    FRM_BRS = 0x04,
    FRM_ESI = 0x08,
} eFrameFlags;

typedef enum {
    MSG_TxFrame = 10,
    MSG_TxEcho,
    MSG_RxFrame,
    MSG_Error,
    MSG_String,
    MSG_Busload,
} eMessageType;

typedef enum {
    FIL_HostClear = 0,
    FIL_HostPass_11,
    FIL_HostPass_29,
} eFilterOperation;

typedef struct {
    uint8_t Operation;
    uint32_t Filter;
    uint32_t Mask;
    uint8_t Index;
    uint8_t DestChannel;
    uint8_t Reserved[6];
} ELMUE_PACKED kFilter;

typedef struct {
    uint32_t prop;
    uint32_t seg1;
    uint32_t seg2;
    uint32_t sjw;
    uint32_t brp;
} ELMUE_PACKED kBitTiming;

typedef struct {
    uint32_t mode;
    uint32_t flags;
} ELMUE_PACKED kDeviceMode;

typedef struct {
    uint32_t seg1_min;
    uint32_t seg1_max;
    uint32_t seg2_min;
    uint32_t seg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} ELMUE_PACKED kTimeMinMax;

typedef struct {
    uint32_t feature;
    uint32_t fclk_can;
    kTimeMinMax time;
} ELMUE_PACKED kCapabilityClassic;

typedef struct {
    uint32_t feature;
    uint32_t fclk_can;
    kTimeMinMax time_nom;
    kTimeMinMax time_data;
} ELMUE_PACKED kCapabilityFD;

typedef struct {
    uint8_t hal_ver_high;
    uint8_t hal_ver_mid;
    uint8_t hal_ver_low;
    uint8_t icount;
    uint32_t sw_version_bcd;
    uint32_t hw_version_bcd;
} ELMUE_PACKED kDeviceVersion;

typedef struct {
    uint16_t McuDeviceID;
    char McuName[25];
    char BoardName[25];
    uint32_t BoardFlags;
} ELMUE_PACKED kBoardInfo;

typedef struct {
    uint8_t size;
    uint8_t msg_type;
} ELMUE_PACKED kHeader;

typedef struct {
    kHeader header;
    uint8_t flags;
    uint32_t can_id;
    uint8_t marker;
} ELMUE_PACKED kTxFrameElmue;

typedef struct {
    kHeader header;
    uint8_t flags;
    uint32_t can_id;
    uint32_t timestamp;
} ELMUE_PACKED kRxFrameElmue;

typedef struct {
    kHeader header;
    uint8_t marker;
    uint32_t timestamp;
} ELMUE_PACKED kTxEchoElmue;

typedef struct {
    kHeader header;
    uint32_t err_id;
    uint8_t err_data[8];
    uint32_t timestamp;
} ELMUE_PACKED kErrorElmue;

typedef struct {
    kHeader header;
    uint8_t bus_load;
} ELMUE_PACKED kBusloadElmue;

}  // namespace elmue_can

#undef ELMUE_PACKED

#endif  // ELMUE_CAN__ELMUE_PROTO_H_
