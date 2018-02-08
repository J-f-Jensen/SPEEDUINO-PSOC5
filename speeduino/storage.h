#ifndef STORAGE_H
#define STORAGE_H

void UpdateStorage(uint16_t addr, uint8_t value);
uint8_t ReadStorage(uint16_t addr);
void writeAllConfig();
void writeConfig(byte tableNum);
void loadConfig();
void loadCalibration();
void writeCalibration();

#define EEPROM_MAX_WRITE_BLOCK 30 //The maximum number of write operations that will be performed in one go. If we try to write to the EEPROM too fast (Each write takes ~3ms) then the rest of the system can hang)
bool eepromWritesPending = false;

/*
Current layout of EEPROM data (Version 3) is as follows (All sizes are in bytes):
|---------------------------------------------------|
|Byte # |Size | Description                         |
|---------------------------------------------------|
| 0     |1    | Data structure version              |
| 1     |2    | X and Y sizes for VE table          |
| 3     |256  | VE Map (16x16)                      |
| 259   |16   | VE Table RPM bins                   |
| 275   |16   | VE Table MAP/TPS bins               |
| 291   |64   | Page 2 settings (Non-Map page)      |
| 355   |2    | X and Y sizes for Ign table         |
| 357   |256  | Ignition Map (16x16)                |
| 613   |16   | Ign Table RPM bins                  |
| 629   |16   | Ign Table MAP/TPS bins              |
| 645   |64   | Page 4 settings (Non-Map page)      |
| 709   |2    | X and Y sizes for AFR table         |
| 711   |256  | AFR Target Map (16x16)              |
| 967   |16   | AFR Table RPM bins                  |
| 983   |16   | AFR Table MAP/TPS bins              |
| 999   |64   | Remaining Page 3 settings           |
| 1063  |64   | Page 4 settings                     |
| 1127  |2    | X and Y sizes for boost table       |
| 1129  |64   | Boost Map (8x8)                     |
| 1193  |8    | Boost Table RPM bins                |
| 1201  |8    | Boost Table TPS bins                |
| 1209  |2    | X and Y sizes                       |
| 1211  |64   | PAGE 8 MAP2                         |
| 1275  |8    | Xbins Map2                          |
| 1283  |8    | Ybins Map2                          |
| 1291  |2    | X and Y sizes1                      |
| 1293  |36   | PAGE 9 MAP1                         |
| 1329  |12   | X and Y Bins1                       |
| 1341  |2    | X and Y size2                       |
| 1343  |36   | PAGE 9 MAP2                         |
| 1379  |6    | X and Y Bins2                       |
| 1391  |2    | X and Y sizes3                      |
| 1393  |36   | PAGE 9 MAP3                         |
| 1429  |6    | X and Y Bins3                       |
| 1441  |2    | X and Y size4                       |
| 1443  |36   | PAGE 9 MAP4                         |
| 1479  |6    | X and Y Bins4                       |
| 1500  |128  | CANBUS config and data (Table 10_)  |
| 1628  |192  | Table 11 - General settings         |
|                                                   |
| 2559  |512  | Calibration data (O2)               |
| 3071  |512  | Calibration data (IAT)              |
| 3583  |512  | Calibration data (CLT)              |
-----------------------------------------------------
*/

#define EEPROM_DATA_VERSION   0

#define EEPROM_CONFIG1_XSIZE  1
#define EEPROM_CONFIG1_YSIZE  2
#define EEPROM_CONFIG1_MAP    3
#define EEPROM_CONFIG1_XBINS  259
#define EEPROM_CONFIG1_YBINS  275
#define EEPROM_CONFIG2_START  291
#define EEPROM_CONFIG2_END    419
#define EEPROM_CONFIG3_XSIZE  419
#define EEPROM_CONFIG3_YSIZE  420
#define EEPROM_CONFIG3_MAP    421
#define EEPROM_CONFIG3_XBINS  677
#define EEPROM_CONFIG3_YBINS  693
#define EEPROM_CONFIG4_START  709
#define EEPROM_CONFIG4_END    837
#define EEPROM_CONFIG5_XSIZE  837
#define EEPROM_CONFIG5_YSIZE  838
#define EEPROM_CONFIG5_MAP    839
#define EEPROM_CONFIG5_XBINS  1095
#define EEPROM_CONFIG5_YBINS  1111
#define EEPROM_CONFIG6_START  1127
#define EEPROM_CONFIG6_END    1255
#define EEPROM_CONFIG8_XSIZE1 1255
#define EEPROM_CONFIG8_YSIZE1 1256
#define EEPROM_CONFIG8_MAP1   1257
#define EEPROM_CONFIG8_XBINS1 1321
#define EEPROM_CONFIG8_YBINS1 1329
#define EEPROM_CONFIG8_XSIZE2 1337
#define EEPROM_CONFIG8_YSIZE2 1338
#define EEPROM_CONFIG8_MAP2   1339
#define EEPROM_CONFIG8_XBINS2 1403
#define EEPROM_CONFIG8_YBINS2 1411
#define EEPROM_CONFIG8_END    1419
#define EEPROM_CONFIG9_XSIZE1 1419
#define EEPROM_CONFIG9_YSIZE1 1420
#define EEPROM_CONFIG9_MAP1   1421
#define EEPROM_CONFIG9_XBINS1 1457
#define EEPROM_CONFIG9_YBINS1 1463
#define EEPROM_CONFIG9_XSIZE2 1469
#define EEPROM_CONFIG9_YSIZE2 1470
#define EEPROM_CONFIG9_MAP2   1471
#define EEPROM_CONFIG9_XBINS2 1507
#define EEPROM_CONFIG9_YBINS2 1513
#define EEPROM_CONFIG9_XSIZE3 1519
#define EEPROM_CONFIG9_YSIZE3 1520
#define EEPROM_CONFIG9_MAP3   1521
#define EEPROM_CONFIG9_XBINS3 1557
#define EEPROM_CONFIG9_YBINS3 1563
#define EEPROM_CONFIG9_XSIZE4 1569
#define EEPROM_CONFIG9_YSIZE4 1570
#define EEPROM_CONFIG9_MAP4   1571
#define EEPROM_CONFIG9_XBINS4 1607
#define EEPROM_CONFIG9_YBINS4 1613
#define EEPROM_CONFIG10_START 1628
#define EEPROM_CONFIG10_END   1756
#define EEPROM_CONFIG11_START 1756
#define EEPROM_CONFIG11_END   1948

#if defined(CORE_PSOC5)
  // In PSOC5 we store the calibration data in flash all other have to be in EEPROM
  #define EEPROM_LAST_BARO    1949

  //Calibration data is stored in flash on the PSOC5 device - Tables need to be initilised

  // O2 default is based on a 14Point7 WB controller
  static const uint8_t CYCODE EEPROM_CALIBRATION_O2[] = {100,  100,  100,  101,  101,  101,  101,  101,  102,  102,  102,  102,  102,  103,  103,  103,  103,  103,  104,  104,  104,  104,  104,  104,  105,  105,  105,  105,  105,  106,  106,  106,  106,  106,  107,  107,  107,  107,  107,  108,  108,  108,  108,  108,  109,  109,  109,  109,  109,  110,  110,  110,  110,  110,  111,  111,  111,  111,  111,  112,  112,  112,  112,  112,  113,  113,  113,  113,  113,  113,  114,  114,  114,  114,  114,  115,  115,  115,  115,  115,  116,  116,  116,  116,  116,  117,  117,  117,  117,  117,  118,  118,  118,  118,  118,  119,  119,  119,  119,  119,  120,  120,  120,  120,  120,  121,  121,  121,  121,  121,  122,  122,  122,  122,  122,  122,  123,  123,  123,  123,  123,  124,  124,  124,  124,  124,  125,  125,  125,  125,  125,  126,  126,  126,  126,  126,  127,  127,  127,  127,  127,  128,  128,  128,  128,  128,  129,  129,  129,  129,  129,  130,  130,  130,  130,  130,  130,  131,  131,  131,  131,  131,  132,  132,  132,  132,  132,  133,  133,  133,  133,  133,  134,  134,  134,  134,  134,  135,  135,  135,  135,  135,  136,  136,  136,  136,  136,  137,  137,  137,  137,  137,  138,  138,  138,  138,  138,  139,  139,  139,  139,  139,  139,  140,  140,  140,  140,  140,  141,  141,  141,  141,  141,  142,  142,  142,  142,  142,  143,  143,  143,  143,  143,  144,  144,  144,  144,  144,  145,  145,  145,  145,  145,  146,  146,  146,  146,  146,  147,  147,  147,  147,  147,  148,  148,  148,  148,  148,  148,  149,  149,  149,  149,  149,  150,  150,  150,  150,  150,  151,  151,  151,  151,  151,  152,  152,  152,  152,  152,  153,  153,  153,  153,  153,  154,  154,  154,  154,  154,  155,  155,  155,  155,  155,  156,  156,  156,  156,  156,  157,  157,  157,  157,  157,  157,  158,  158,  158,  158,  158,  159,  159,  159,  159,  159,  160,  160,  160,  160,  160,  161,  161,  161,  161,  161,  162,  162,  162,  162,  162,  163,  163,  163,  163,  163,  164,  164,  164,  164,  164,  165,  165,  165,  165,  165,  165,  166,  166,  166,  166,  166,  167,  167,  167,  167,  167,  168,  168,  168,  168,  168,  169,  169,  169,  169,  169,  170,  170,  170,  170,  170,  171,  171,  171,  171,  171,  172,  172,  172,  172,  172,  173,  173,  173,  173,  173,  174,  174,  174,  174,  174,  174,  175,  175,  175,  175,  175,  176,  176,  176,  176,  176,  177,  177,  177,  177,  177,  178,  178,  178,  178,  178,  179,  179,  179,  179,  179,  180,  180,  180,  180,  180,  181,  181,  181,  181,  181,  182,  182,  182,  182,  182,  183,  183,  183,  183,  183,  183,  184,  184,  184,  184,  184,  185,  185,  185,  185,  185,  186,  186,  186,  186,  186,  187,  187,  187,  187,  187,  188,  188,  188,  188,  188,  189,  189,  189,  189,  189,  190,  190,  190,  190,  190,  191,  191,  191,  191,  191,  191,  192,  192,  192,  192,  192,  193,  193,  193,  193,  193,  194,  194,  194,  194,  194,  195,  195,  195,  195,  195,  196,  196,  196,  196,  196,  197,  197,  197,  197,  197,  198,  198,  198,  198,  198,  199,  199,  199,  199,  199,  200,  200,  200};

  // IAT defaults is based on a SAAB(BOSCH) sensor
  static const uint8_t CYCODE EEPROM_CALIBRATION_IAT[] = {61, 61, 61, 61, 61, 61, 216, 208, 202, 196, 191, 187, 183, 180, 176, 173, 171, 168, 166, 163, 161, 159, 157, 156, 153, 152, 151, 149, 147, 146, 145, 143, 142, 141, 140, 139, 138, 137, 136, 135, 133, 133, 132, 131, 130, 129, 128, 127, 127, 126, 125, 125, 123, 123, 122, 121, 121, 120, 120, 119, 118, 117, 117, 116, 116, 115, 115, 114, 113, 113, 112, 112, 111, 111, 110, 110, 110, 109, 108, 108, 107, 107, 106, 106, 106, 105, 105, 104, 104, 103, 103, 102, 102, 102, 101, 101, 101, 100, 100, 100, 99, 98, 98, 98, 97, 97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 93, 93, 93, 93, 92, 92, 92, 91, 91, 91, 91, 90, 90, 90, 89, 89, 88, 88, 88, 88, 87, 87, 87, 87, 86, 86, 86, 86, 85, 85, 85, 85, 84, 84, 83, 83, 83, 83, 82, 82, 82, 82, 81, 81, 81, 81, 81, 80, 80, 80, 80, 79, 79, 78, 78, 78, 78, 78, 77, 77, 77, 77, 77, 76, 76, 76, 76, 75, 75, 75, 75, 75, 74, 74, 74, 73, 73, 73, 73, 73, 72, 72, 72, 72, 72, 71, 71, 71, 71, 71, 70, 70, 70, 70, 70, 69, 69, 68, 68, 68, 68, 68, 68, 67, 67, 67, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 60, 60, 60, 60, 60, 60, 59, 59, 58, 58, 58, 58, 58, 58, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 55, 55, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 53, 53, 53, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 49, 49, 49, 48, 48, 48, 48, 48, 48, 47, 47, 47, 47, 47, 46, 46, 46, 46, 46, 46, 45, 45, 45, 45, 45, 44, 44, 44, 43, 43, 43, 43, 43, 43, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 38, 38, 38, 38, 38, 37, 37, 37, 37, 37, 36, 36, 36, 35, 35, 35, 35, 34, 34, 34, 34, 34, 33, 33, 33, 33, 33, 32, 32, 32, 32, 31, 31, 31, 30, 30, 30, 30, 29, 29, 29, 29, 28, 28, 28, 28, 27, 27, 27, 27, 27, 26, 26, 25, 25, 25, 24, 24, 24, 24, 23, 23, 23, 23, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61};

  // CLT defaults is based on a SAAB(BOSCH) sensor
  static const uint8_t CYCODE EEPROM_CALIBRATION_CLT[] = {122, 122, 122, 122, 122, 122, 216, 208, 202, 196, 191, 187, 183, 180, 176, 173, 171, 168, 166, 163, 161, 159, 157, 156, 153, 152, 151, 149, 147, 146, 145, 143, 142, 141, 140, 139, 138, 137, 136, 135, 133, 133, 132, 131, 130, 129, 128, 127, 127, 126, 125, 125, 123, 123, 122, 121, 121, 120, 120, 119, 118, 117, 117, 116, 116, 115, 115, 114, 113, 113, 112, 112, 111, 111, 110, 110, 110, 109, 108, 108, 107, 107, 106, 106, 106, 105, 105, 104, 104, 103, 103, 102, 102, 102, 101, 101, 101, 100, 100, 100, 99, 98, 98, 98, 97, 97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 93, 93, 93, 93, 92, 92, 92, 91, 91, 91, 91, 90, 90, 90, 89, 89, 88, 88, 88, 88, 87, 87, 87, 87, 86, 86, 86, 86, 85, 85, 85, 85, 84, 84, 83, 83, 83, 83, 82, 82, 82, 82, 81, 81, 81, 81, 81, 80, 80, 80, 80, 79, 79, 78, 78, 78, 78, 78, 77, 77, 77, 77, 77, 76, 76, 76, 76, 75, 75, 75, 75, 75, 74, 74, 74, 73, 73, 73, 73, 73, 72, 72, 72, 72, 72, 71, 71, 71, 71, 71, 70, 70, 70, 70, 70, 69, 69, 68, 68, 68, 68, 68, 68, 67, 67, 67, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 64, 64, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 60, 60, 60, 60, 60, 60, 59, 59, 58, 58, 58, 58, 58, 58, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 55, 55, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 53, 53, 53, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 49, 49, 49, 48, 48, 48, 48, 48, 48, 47, 47, 47, 47, 47, 46, 46, 46, 46, 46, 46, 45, 45, 45, 45, 45, 44, 44, 44, 43, 43, 43, 43, 43, 43, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 38, 38, 38, 38, 38, 37, 37, 37, 37, 37, 36, 36, 36, 35, 35, 35, 35, 34, 34, 34, 34, 34, 33, 33, 33, 33, 33, 32, 32, 32, 32, 31, 31, 31, 30, 30, 30, 30, 29, 29, 29, 29, 28, 28, 28, 28, 27, 27, 27, 27, 27, 26, 26, 25, 25, 25, 24, 24, 24, 24, 23, 23, 23, 23, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122};

#else
  //Calibration data is stored at the end of the EEPROM (This is in case any further calibration tables are needed as they are large blocks)
  #define EEPROM_LAST_BARO      2558
  #define EEPROM_CALIBRATION_O2 2559
  #define EEPROM_CALIBRATION_IAT 3071
  #define EEPROM_CALIBRATION_CLT 3583
#endif

#endif // STORAGE_H
