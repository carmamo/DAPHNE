#include "sin_model_data.h"

ai_handle ai_sin_model_data_weights_get(void)
{

  AI_ALIGNED(4)
  static const ai_u8 s_sin_model_weights[ 1284 ] = {
    0xc9, 0x68, 0x64, 0x3e, 0x78, 0xb8, 0x4a, 0x3e, 0x31, 0xc4,
    0xa3, 0x3e, 0x66, 0x97, 0x73, 0xbe, 0x44, 0xea, 0xf3, 0xbd,
    0xb1, 0xcb, 0xe1, 0x3e, 0x57, 0x9a, 0x82, 0xbe, 0xa6, 0xf1,
    0xb8, 0x3e, 0x70, 0x5b, 0xd0, 0x3e, 0xb8, 0xed, 0xb8, 0xbd,
    0xa3, 0xf0, 0x10, 0xbf, 0xd2, 0x99, 0x36, 0xbe, 0x8b, 0x6c,
    0x99, 0x3e, 0xfb, 0xc1, 0xca, 0xbe, 0x58, 0x2f, 0xaa, 0xbd,
    0x40, 0x10, 0xff, 0xbc, 0xe7, 0xdb, 0x11, 0xbf, 0x03, 0xa0,
    0x1b, 0x3f, 0x58, 0x46, 0x61, 0x3d, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xe1, 0x0e, 0x95, 0xbe, 0x00, 0x00,
    0x00, 0x00, 0xf6, 0xa9, 0x9e, 0x3e, 0x46, 0x2c, 0xec, 0xbc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x8e, 0xcf, 0x04, 0xbf, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x30,
    0xa9, 0xbf, 0xe4, 0xe2, 0x08, 0x3f, 0xc1, 0x4b, 0x3a, 0x3e,
    0x91, 0x2c, 0x93, 0x3e, 0xd0, 0x87, 0x91, 0xbd, 0xdb, 0x9e,
    0x4d, 0xbe, 0x8a, 0x3a, 0x29, 0x3e, 0x92, 0x9e, 0xbb, 0x3e,
    0x7f, 0x09, 0x4a, 0xbe, 0xf4, 0x22, 0x8a, 0xbd, 0x20, 0xb9,
    0x73, 0x3d, 0x70, 0x3a, 0x72, 0xbd, 0x94, 0x6f, 0x8e, 0xbf,
    0x02, 0x14, 0x70, 0x3e, 0x27, 0x0f, 0xc2, 0xbe, 0x9e, 0x76,
    0x65, 0x3e, 0xcd, 0xfc, 0xd2, 0x3e, 0x78, 0x58, 0x7a, 0x3e,
    0xb3, 0x8b, 0x59, 0x3e, 0x73, 0x78, 0xda, 0x3e, 0x28, 0xd3,
    0x8b, 0xbd, 0xca, 0x74, 0xcb, 0xbe, 0x7a, 0x83, 0x30, 0x3e,
    0x9a, 0x25, 0xa5, 0xbe, 0xea, 0x47, 0x36, 0xbe, 0xa1, 0x73,
    0x8c, 0x3e, 0xe2, 0x21, 0xa8, 0xbe, 0x58, 0xac, 0x4a, 0xbd,
    0x48, 0x09, 0xbf, 0xbc, 0x81, 0xaa, 0xbe, 0xbe, 0x00, 0xa3,
    0x31, 0x3d, 0x00, 0x39, 0x9b, 0x3a, 0xf7, 0xeb, 0xaf, 0x3e,
    0x25, 0xa4, 0xd2, 0xbe, 0x1a, 0x42, 0xc2, 0x3d, 0x91, 0x71,
    0xbc, 0xbe, 0x5c, 0x5d, 0xef, 0xbd, 0x7e, 0x66, 0xb6, 0xbe,
    0xa0, 0xe4, 0x39, 0xbd, 0xfe, 0xc6, 0x80, 0xbe, 0xf3, 0x7e,
    0x9d, 0xbe, 0x0c, 0x20, 0x9f, 0xbd, 0x6e, 0x72, 0x4d, 0x3e,
    0xd8, 0x7f, 0x81, 0xbe, 0x94, 0x66, 0xd6, 0x3e, 0x50, 0x73,
    0x3f, 0x3d, 0xa2, 0x06, 0x19, 0xbe, 0xce, 0x1e, 0x36, 0x3e,
    0xb7, 0x13, 0x18, 0xbf, 0xc0, 0x66, 0xed, 0x3e, 0xb5, 0x82,
    0xcd, 0x3e, 0x4a, 0x6a, 0x9b, 0xbe, 0xc4, 0x2d, 0x8c, 0xbd,
    0xbe, 0xd4, 0xbe, 0xbe, 0x56, 0xd3, 0x4a, 0x3e, 0xc2, 0xb7,
    0xb8, 0x3e, 0x0f, 0x99, 0x71, 0x3e, 0x00, 0x0b, 0xbc, 0x3b,
    0xa4, 0x03, 0xd4, 0x3d, 0xc0, 0xf4, 0xb4, 0x3c, 0xc8, 0x3a,
    0x88, 0xbe, 0xdf, 0x6e, 0x9d, 0x3e, 0x70, 0x4b, 0x89, 0x3c,
    0xb9, 0x80, 0xd7, 0x3e, 0xa4, 0x53, 0x36, 0x3c, 0xbf, 0xdf,
    0xbd, 0xbe, 0x74, 0xf1, 0x31, 0xbd, 0xac, 0x86, 0x77, 0xbe,
    0xc1, 0x8c, 0xb0, 0x3e, 0x25, 0xcd, 0x68, 0xbe, 0xcf, 0xf3,
    0x83, 0x3e, 0xdd, 0x01, 0x0b, 0x3e, 0x5f, 0x32, 0xd0, 0x3d,
    0xde, 0x28, 0x18, 0x3e, 0xe0, 0xa1, 0x47, 0xbd, 0x4d, 0xb7,
    0xc5, 0xbe, 0xe2, 0x25, 0x74, 0x3e, 0xf4, 0x8c, 0xab, 0x3d,
    0x09, 0xcc, 0x94, 0x3e, 0x3e, 0x20, 0xb2, 0xbe, 0xb2, 0xef,
    0x0c, 0xbf, 0x1e, 0xec, 0xe5, 0x3e, 0x03, 0xa5, 0xa0, 0xbe,
    0xc7, 0x0e, 0xa9, 0x3e, 0x01, 0xe0, 0xd2, 0x3e, 0x7d, 0x42,
    0x5a, 0x3e, 0x54, 0xf7, 0x78, 0xbe, 0xcb, 0x85, 0xef, 0xbd,
    0xa8, 0x90, 0xd8, 0x3d, 0xdf, 0xdf, 0x5a, 0xbe, 0xb4, 0xc7,
    0x9d, 0x3d, 0xcc, 0x92, 0x96, 0xbd, 0xe3, 0x0b, 0x9e, 0xbe,
    0x08, 0x70, 0x21, 0x3d, 0xd4, 0x29, 0x93, 0x3d, 0x8f, 0x12,
    0xac, 0xbe, 0xb0, 0x87, 0x49, 0x3e, 0xc8, 0x6f, 0x95, 0x3d,
    0x01, 0xca, 0x8b, 0x3e, 0xe4, 0xb6, 0xaa, 0x3d, 0xaa, 0x46,
    0xd5, 0xbe, 0x88, 0xad, 0x59, 0x3e, 0xf4, 0x92, 0xf3, 0x3d,
    0x00, 0x9e, 0xcc, 0x3d, 0xf4, 0x13, 0x8e, 0x3c, 0x28, 0x8d,
    0x99, 0xbd, 0xee, 0x18, 0x71, 0x3e, 0xa5, 0x1d, 0x88, 0x3e,
    0x91, 0x2c, 0x47, 0xbd, 0x35, 0xba, 0x91, 0x3e, 0x3f, 0x52,
    0x81, 0x3e, 0x50, 0xe1, 0x18, 0xbe, 0x90, 0x71, 0x44, 0x3e,
    0x7d, 0xfd, 0xff, 0xbd, 0x05, 0xb6, 0x9f, 0xbc, 0x81, 0x72,
    0xc2, 0xbe, 0xbb, 0x32, 0x82, 0x3e, 0xa0, 0x06, 0xb7, 0x3e,
    0x0a, 0x1e, 0x0c, 0x3e, 0x0d, 0x22, 0x17, 0xbf, 0xda, 0xe3,
    0x19, 0xbf, 0xd5, 0x51, 0x80, 0xbe, 0xf4, 0x98, 0x55, 0xbe,
    0xf9, 0xc4, 0xa9, 0x3e, 0xed, 0xae, 0x1b, 0x3e, 0x72, 0x8d,
    0x21, 0xbe, 0xca, 0xad, 0x45, 0x3e, 0x94, 0x3e, 0x6d, 0xbe,
    0x61, 0x8a, 0xd1, 0xbe, 0x4b, 0xa2, 0x5c, 0xbe, 0xc4, 0x79,
    0x83, 0xbe, 0xaa, 0x18, 0x93, 0xbe, 0x70, 0xf1, 0x91, 0x3c,
    0x13, 0x48, 0xa2, 0x3e, 0x44, 0x9b, 0xcc, 0x3d, 0xb1, 0x45,
    0x7d, 0xbd, 0x77, 0x54, 0x23, 0xbf, 0xc9, 0x9f, 0x9c, 0x3e,
    0xe0, 0xa5, 0x15, 0xbc, 0x1a, 0xd1, 0xa0, 0xbe, 0x46, 0x7b,
    0x19, 0xbf, 0x6e, 0x84, 0x28, 0x3e, 0x9b, 0x52, 0x83, 0xbe,
    0x64, 0x6e, 0xcf, 0xbe, 0xd7, 0x2c, 0x90, 0xbf, 0x87, 0xa0,
    0xf8, 0xbd, 0xee, 0x08, 0x16, 0x3e, 0xb2, 0x23, 0x6e, 0xbe,
    0xef, 0xf9, 0xa4, 0xbe, 0x36, 0x21, 0x5d, 0x3e, 0x1c, 0xc4,
    0xdf, 0x3d, 0x96, 0x3d, 0x9e, 0xbe, 0x0c, 0x16, 0x86, 0x3e,
    0x44, 0xd2, 0xb2, 0x3d, 0x26, 0xb7, 0x0b, 0xbe, 0x12, 0x25,
    0x75, 0xbe, 0x3e, 0x8b, 0x2e, 0xbf, 0x4a, 0xca, 0x16, 0x3e,
    0x7c, 0xa9, 0x52, 0xbe, 0x9b, 0x9a, 0xc1, 0x3e, 0x79, 0xcc,
    0x93, 0xbc, 0xe1, 0x08, 0x96, 0xbe, 0x43, 0xb1, 0xa6, 0xbe,
    0x0a, 0xe5, 0x7e, 0x3e, 0x64, 0x62, 0xda, 0x3d, 0xee, 0x77,
    0x7e, 0x3d, 0x88, 0xac, 0x4d, 0xbe, 0x86, 0x66, 0x79, 0x3e,
    0xc8, 0xab, 0xa5, 0xbe, 0x3f, 0x32, 0xac, 0x3e, 0xf4, 0x24,
    0xc9, 0x3d, 0xa6, 0x5f, 0x22, 0x3e, 0x57, 0x9e, 0xfb, 0x3d,
    0xc6, 0x04, 0x25, 0x3e, 0xe8, 0x39, 0x48, 0xbe, 0x34, 0xc0,
    0xd3, 0xbd, 0xae, 0x51, 0x38, 0xbf, 0x13, 0x75, 0x14, 0xbe,
    0xb4, 0x14, 0xdb, 0x3e, 0x7f, 0x0e, 0xdd, 0xbe, 0x0f, 0x36,
    0xa8, 0x3e, 0x5e, 0x83, 0x7c, 0xbe, 0xf9, 0x0c, 0x86, 0x3e,
    0xbd, 0x7b, 0xe0, 0x3e, 0x26, 0x37, 0xe9, 0xbd, 0x0f, 0x4f,
    0x80, 0x3e, 0x79, 0x21, 0x96, 0x3e, 0xf3, 0x3a, 0xb1, 0x3e,
    0xe5, 0xba, 0x39, 0xbd, 0x2c, 0x64, 0x32, 0xbe, 0xd0, 0x7c,
    0x66, 0xbd, 0x35, 0x55, 0xb2, 0x3e, 0x12, 0x84, 0x55, 0xbf,
    0xb3, 0x81, 0x8f, 0x3e, 0x52, 0x88, 0xd2, 0x3d, 0xac, 0xe6,
    0xa3, 0x3d, 0x7e, 0x06, 0x02, 0x3e, 0xa4, 0xce, 0x16, 0xbe,
    0xe4, 0x0e, 0xb1, 0x3d, 0x40, 0x49, 0xbd, 0x3e, 0x56, 0xe7,
    0x6d, 0x3d, 0x3f, 0xbf, 0xae, 0xbe, 0xc0, 0x40, 0xa8, 0xbb,
    0x46, 0xcf, 0xad, 0xbe, 0x50, 0x02, 0x24, 0xbf, 0x7d, 0x1c,
    0xdd, 0xbe, 0x88, 0x33, 0x30, 0xbe, 0x4d, 0x64, 0x85, 0x3e,
    0x99, 0xa8, 0x8c, 0x3e, 0xd7, 0x14, 0xbd, 0xbe, 0xdb, 0xd1,
    0xb5, 0x3e, 0x10, 0x3c, 0xfd, 0xbc, 0xd0, 0x3b, 0x52, 0xbd,
    0x24, 0x5c, 0xdd, 0xbd, 0xde, 0x88, 0x35, 0xbe, 0x67, 0x08,
    0xc1, 0xbc, 0x5e, 0xcd, 0xc1, 0x3e, 0xbe, 0x76, 0xd0, 0xbe,
    0x48, 0xcb, 0x60, 0x3d, 0xab, 0xa9, 0x95, 0x3e, 0x66, 0x2d,
    0x7c, 0x3e, 0xbb, 0x19, 0xd3, 0x3e, 0xd8, 0x43, 0x6b, 0x3d,
    0xf1, 0x06, 0xdc, 0xbe, 0x78, 0xad, 0x66, 0xbe, 0x92, 0x49,
    0x4e, 0xbe, 0xe0, 0x26, 0xa4, 0xbe, 0x6d, 0xb0, 0x4a, 0xbe,
    0xc7, 0xac, 0xa9, 0x3e, 0x54, 0xf8, 0x83, 0xbb, 0xb1, 0x06,
    0xd1, 0x3e, 0x29, 0x5f, 0x49, 0x3e, 0x43, 0x27, 0xc4, 0x3d,
    0xf1, 0xda, 0x84, 0x3e, 0x70, 0xe2, 0x38, 0x3d, 0x79, 0x8e,
    0x59, 0xbe, 0xdc, 0x57, 0x35, 0x3c, 0x5e, 0xc9, 0xc0, 0xbe,
    0xe0, 0xd6, 0x1e, 0xbc, 0x02, 0xe4, 0x43, 0x3e, 0x99, 0xb4,
    0x90, 0x3e, 0x18, 0xdc, 0xa2, 0x3e, 0x3e, 0x05, 0xc0, 0x3e,
    0x9c, 0x42, 0x08, 0xbe, 0x3e, 0x84, 0x35, 0xbe, 0xfd, 0x12,
    0xd5, 0xbd, 0x06, 0xee, 0x4d, 0x3e, 0x65, 0x9e, 0xa0, 0x3e,
    0x4e, 0xd5, 0x62, 0xbe, 0x29, 0x1e, 0x73, 0xbe, 0x1a, 0x4e,
    0x81, 0xbe, 0x8c, 0x3d, 0xb6, 0xbe, 0xcd, 0xf2, 0x6d, 0x3d,
    0xb4, 0x83, 0xdf, 0xbd, 0x39, 0x6f, 0xcb, 0xbe, 0x88, 0x8e,
    0xcd, 0xbe, 0x86, 0x23, 0x16, 0x3f, 0xf5, 0x3c, 0x86, 0xbd,
    0xb3, 0x3a, 0x4f, 0xbb, 0x2d, 0xfe, 0x49, 0x3e, 0x34, 0x7c,
    0x58, 0x3a, 0x39, 0xaf, 0x62, 0x3e, 0x42, 0x75, 0xb7, 0xbe,
    0x34, 0x18, 0x2c, 0x3f, 0x4e, 0xaa, 0x14, 0x3f, 0x2b, 0x54,
    0x1a, 0x3f, 0xde, 0x7c, 0x97, 0xbc, 0x80, 0xe2, 0x6a, 0x3e,
    0xfb, 0xb9, 0x56, 0x3e, 0x2a, 0xe3, 0x07, 0xbf, 0xfe, 0x3d,
    0x4f, 0xbb, 0x48, 0xe3, 0xc6, 0xbe, 0x8e, 0xf5, 0xf1, 0x3f,
    0xf0, 0xe3, 0xb5, 0x3e, 0x62, 0x63, 0xf3, 0x3e, 0x3c, 0xd7,
    0x10, 0xbf, 0x7f, 0x44, 0x97, 0xbd, 0xd6, 0xa3, 0x0f,
    0xbf, 0x27, 0x41, 0x00, 0x3f, 0x8f, 0x8e, 0x9e, 0xbf,
    0x5c, 0xb1, 0x8b, 0xbf, 0x7a, 0xe9, 0xbb, 0x3f, 0xf9,
    0xf1, 0xf8, 0x3d, 0x29, 0x0d, 0x27, 0xbf, 0x36, 0xa2,
    0x3e, 0xbf, 0xb2, 0x24, 0x62, 0x3f, 0xc1, 0xe9, 0x94,
    0x3e, 0x19, 0x53, 0x7c, 0x3e, 0x77, 0x00, 0xdf, 0xbe
  };

  return AI_HANDLE_PTR(s_sin_model_weights);

}

