// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#include "targetver.h"

#include "UDPTrans.h"
#include <Windows.h>

#include <stdio.h>
#include <tchar.h>
#include <time.h>

#include <conio.h>
#include <string>
//#include <iostream>
//#include <fstream>

// TODO: プログラムに必要な追加ヘッダーをここで参照してください。
// data structure for shared memory
#include "../../Common/Shrmem_struct.h"
#include "../../Common/AssistShare.h"

// shared memory manager
#include "../../Common/RTCLib/cpp/header/SharedMemoryManager.h"
#include "../../Common/RTCLib/cpp/header/CSVLoader.h"
#include "../../Common/RTCLib/cpp/header/StopWatch.h"
#include "../../Common/RTCLib/cpp/header/DataStorage.h"
#include "setting.h"

#include "ShrMemManager.h"


#include <math.h>
#include <Eigen/Dense>
#include <stdlib.h>

// for carsim
#include "carsim/CarsimCar2014.h"
#include "setting.h"
