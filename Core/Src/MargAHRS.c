/*
 * MargAHRS.c
 *
 *  Created on: Jan 20, 2025
 *      Author: nino
 */
///=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//
//  1 June 2012 Modified by J. Ihlein
// 27 Aug  2012 Extensively modified to include G.K. Egan's accel confidence calculations and
//                                                          calculation efficiency updates
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "board.h"
#include <math.h>
//----------------------------------------------------------------------------------------------------
// Variable definitions

float exAcc    = 0.0f,    eyAcc = 0.0f,    ezAcc = 0.0f; // accel error
float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f; // accel integral error

float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f; // mag error
float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f; // mag integral error

float kpAcc, kiAcc;

//float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// auxiliary variables to reduce number of repeated operations
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

float halfT;

uint8_t MargAHRSinitialized = false;
//----------------------------------------------------------------------------------------------------
float margAttitude500Hz[3];

float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;

#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)

float constrain(float input, float minValue, float maxValue)
{
    if (input < minValue)
        return minValue;
    else if (input > maxValue)
        return maxValue;
    else
        return input;
}


void calculateAccConfidence(float accMag)
{
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity

    static float accMagP = 1.0f;

    accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

    accMag  = HardFilter(accMagP, accMag);
    accMagP = accMag;

    accConfidence = constrain(1.0f - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);
}

//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Initialization
//====================================================================================================
//航姿參考系統初始化
void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

	//使用加速度數據計算歐拉角 ，滾轉角和俯仰角
    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

	//對歐拉角進行餘弦和正弦計算，分別把計算結果儲存下來
    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

//    magX = 1.0f;  // HJI mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

//    magY = 0.0f;  // HJI my * cosRoll - mz * sinRoll;

//    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
//    magY = my * cosRoll - mz * sinRoll;

//    initialHdg = atan2f(-magY, magX);//解算航向角

    initialHdg = atan2f(my, mx);//解算航向角

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);
printf("initialRoll=%f, initialPitch=%f, initialHdg=%f \r\n", initialRoll*57.3,initialPitch*57.3,initialHdg*57.3);
		//得到四元數
    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

		//把計算參考方向用到的值先都計算好,減少重複計算,因為MargAHRSupdate函式裡面要用到。
    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

//====================================================================================================
// Function
//====================================================================================================
//航姿參考系統更新
/*void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;

    //-------------------------------------------

    if ((MargAHRSinitialized == false)) // HJI && (magDataUpdate == true))
    {
			//如果航姿參考系統參數還沒有初始化過，那麼執行AHRS初始化
        MargAHRSinit(ax, ay, az, mx, my, mz);

        MargAHRSinitialized = true;//標記航姿參考系統參數已經初始化過
    }

    //-------------------------------------------

    if (MargAHRSinitialized == true)//如果航姿參考系統參數已經初始化過
    {
        halfT = dt * 0.5f;//半週期，求解四元數微分方程時用得到。

        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//加速度歸一化

        if (norm != 0.0f)//如果歸一化后的模等於0 ，那麼說明加速度數據或者感測器不正常，正常情況下 歸一化后的結果恒等於 1.0 ，這是重點。
        {
            calculateAccConfidence(norm);//由於處於運動狀態，所有要計算加速度數據歸一化后的可信度
//            kpAcc = eepromConfig.KpAcc * accConfidence; //加速度比例係數 * 可信度
//            kiAcc = eepromConfig.KiAcc * accConfidence;//加速度積分系數 * 可信度
            kpAcc = 5.0 * accConfidence; //加速度比例係數 * 可信度
            kiAcc = 0.0 * accConfidence;//加速度積分系數 * 可信度

            normR = 1.0f / norm; //加速度歸一化
            ax *= normR;
            ay *= normR;
            az *= normR;

            // estimated direction of gravity (v)
//0227            vx = 2.0f * (q1q3 - q0q2);//計算方向餘弦矩陣
//            vy = 2.0f * (q0q1 + q2q3);
//            vz = q0q0 - q1q1 - q2q2 + q3q3;
            vx = (q1q3 - q0q2);//計算方向餘弦矩陣
            vy = (q0q1 + q2q3);
            vz = q0q0 - 0.5f + q3q3;
            // error is sum of cross product between reference direction
            // of fields and direction measured by sensors
					//誤差是由感測器測量的參考方向與方向之間的叉積,由此
					//得到一個誤差向量，通過這個誤差向量來修正陀螺儀數據。
//0227            exAcc = vy * az - vz * ay;
//            eyAcc = vz * ax - vx * az;
//            ezAcc = vx * ay - vy * ax;
            exAcc += vy * az - vz * ay;
            eyAcc += vz * ax - vx * az;
            ezAcc += vx * ay - vy * ax;

            gx += exAcc * kpAcc;//比例增益控制加速度計的收斂速度
            gy += eyAcc * kpAcc;
            gz += ezAcc * kpAcc;

            if (kiAcc > 0.0f)//用積分增益控制陀螺儀的偏差收斂速率
            {
                exAccInt += exAcc * kiAcc;
                eyAccInt += eyAcc * kiAcc;
                ezAccInt += ezAcc * kiAcc;

                gx += exAccInt;
                gy += eyAccInt;
                gz += ezAccInt;
            }
        }

        //-------------------------------------------

        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//三軸磁力計歸一化

        if ((magDataUpdate == true) && (norm != 0.0f))//如果入口參數magDataUpdate == true並且歸一化的結果norm不是0，才對磁力計數據進行更新計算
        {
            normR = 1.0f / norm;//三軸磁場歸一化
            mx *= normR;
            my *= normR;
            mz *= normR;

            // compute reference direction of flux
					//計算參考方向
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));

            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));

//0227            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
            hz = 2.0f * (mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2));

            bx = sqrt((hx * hx) + (hy * hy));

            bz = hz;

            // estimated direction of flux (w)
					//根據參考方向估計雲臺機體方向
//0227            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));

//0227            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));

//0227            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

//0227            exMag = my * wz - mz * wy;//三軸磁場和估計方向進行叉積運算,計算估計方向與三軸磁場的偏差
//0227            eyMag = mz * wx - mx * wz;
//0227            ezMag = mx * wy - my * wx;

            wx = (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));

            wy = (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));

            wz = (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

            exMag += my * wz - mz * wy;//三軸磁場和估計方向進行叉積運算
            eyMag += mz * wx - mx * wz;
            ezMag += mx * wy - my * wx;
            // use un-extrapolated old values between magnetometer updates
            // dubious as dT does not apply to the magnetometer calculation so
            // time scaling is embedded in KpMag and KiMag
						//使用估計的舊值與磁力計值進行更新，dT不能應用在磁力計計算中，因此時間被嵌入在KpMag 和 KiMag裡面
            //            gx += exMag * eepromConfig.KpMag;//比例增益控制磁強計收斂速度
            //            gy += eyMag * eepromConfig.KpMag;
            //            gz += ezMag * eepromConfig.KpMag;
            gx += exMag * 2.0;//比例增益控制磁強計收斂速度
            gy += eyMag * 2.0;
            gz += ezMag * 2.0;

//            if (eepromConfig.KiMag > 0.0f)//用積分增益控制陀螺儀的偏差收斂速率
//            {
            	 //               exMagInt += exMag * eepromConfig.KiMag;
            	 //               eyMagInt += eyMag * eepromConfig.KiMag;
            	 //               ezMagInt += ezMag * eepromConfig.KiMag;
//            	                exMagInt += exMag * 0.0;
//            	                eyMagInt += eyMag * 0.0;
//            	                ezMagInt += ezMag * 0.0;

//                gx += exMagInt;
//                gy += eyMagInt;
//                gz += ezMagInt;
//            }

        }

        //-------------------------------------------

        // integrate quaternion rate
			 //四元數微分方程，其中halfT為測量週期，g為陀螺儀角速度，其餘都是已知量，這裡使用了一階龍格庫塔法求解四元數微分方程。
//0227        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
//0227        q1i = (q0 * gx + q2 * gz - q3 * gy) * halfT;
//0227        q2i = (q0 * gy - q1 * gz + q3 * gx) * halfT;
//0227        q3i = (q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0i = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1i = 0.5f * (q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2i = 0.5f * (q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3i = 0.5f * (q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0 += q0i;
        q1 += q1i;
        q2 += q2i;
        q3 += q3i;

        // normalise quaternion
				//四元數歸一化，為什麼又要歸一化呢？這是因為引入了誤差向量后四元數失去了規範性了(模不等於1了),所以要重新歸一化
        normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normR;
        q1 *= normR;
        q2 *= normR;
        q3 *= normR;

        // auxiliary variables to reduce number of repeated operations
				//把計算參考方向用到的值先都計算好,減少下面計算歐拉角時候的重複計算。
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        float margAttitude500Hz[3] = {0.0f, 0.0f, 0.0f};
				//最後根據四元數方向餘弦陣和歐拉角的轉換關係，把四元數轉換成歐拉角
        margAttitude500Hz[0] = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3)*57.296;
        margAttitude500Hz[1] = -asinf(2.0f * (q1q3 - q0q2))*57.296;
        margAttitude500Hz[2] = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)*57.296;

        printf("Roll=%f, Pitch=%f, Yaw=%f \r\n", margAttitude500Hz[0],margAttitude500Hz[1], margAttitude500Hz[2]);
    }
}*/
void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;

    //-------------------------------------------

    if ((MargAHRSinitialized == false)) // HJI && (magDataUpdate == true))
    {
			//如果航姿參考系統參數還沒有初始化過，那麼執行AHRS初始化
        MargAHRSinit(ax, ay, az, mx, my, mz);

        MargAHRSinitialized = true;//標記航姿參考系統參數已經初始化過
    }

    //-------------------------------------------

    if (MargAHRSinitialized == true)//如果航姿參考系統參數已經初始化過
    {
        halfT = dt * 0.5f;//半週期，求解四元數微分方程時用得到。

        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//加速度歸一化

        if (norm != 0.0f)//如果歸一化后的模等於0 ，那麼說明加速度數據或者感測器不正常，正常情況下 歸一化后的結果恒等於 1.0 ，這是重點。
        {
            calculateAccConfidence(norm);//由於處於運動狀態，所有要計算加速度數據歸一化后的可信度
            kpAcc = 5.0f * accConfidence; //加速度比例係數 * 可信度
            kiAcc = 0.0f * accConfidence;//加速度積分系數 * 可信度

            normR = 1.0f / norm; //加速度歸一化
            ax *= normR;
            ay *= normR;
            az *= normR;

            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3 - q0q2);//計算方向餘弦矩陣
            vy = 2.0f * (q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;

            // error is sum of cross product between reference direction
            // of fields and direction measured by sensors
					//誤差是由感測器測量的參考方向與方向之間的叉積,由此
					//得到一個誤差向量，通過這個誤差向量來修正陀螺儀數據。
            exAcc = vy * az - vz * ay;
            eyAcc = vz * ax - vx * az;
            ezAcc = vx * ay - vy * ax;


            gx += exAcc * kpAcc;//比例增益控制加速度計的收斂速度
            gy += eyAcc * kpAcc;
            gz += ezAcc * kpAcc;

            if (kiAcc > 0.0f)//用積分增益控制陀螺儀的偏差收斂速率
            {
                exAccInt += exAcc * kiAcc;
                eyAccInt += eyAcc * kiAcc;
                ezAccInt += ezAcc * kiAcc;

                gx += exAccInt;
                gy += eyAccInt;
                gz += ezAccInt;
            }
        }

        //-------------------------------------------

        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//三軸磁力計歸一化

        if ((magDataUpdate == true) && (norm != 0.0f))//如果入口參數magDataUpdate == true並且歸一化的結果norm不是0，才對磁力計數據進行更新計算
        {
            normR = 1.0f / norm;//三軸磁場歸一化
            mx *= normR;
            my *= normR;
            mz *= normR;

            // compute reference direction of flux
					//計算參考方向
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));

            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));

            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            bx = sqrt((hx * hx) + (hy * hy));

            bz = hz;

            // estimated direction of flux (w)
					//根據參考方向估計雲臺機體方向
            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));

            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));

            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

            exMag = my * wz - mz * wy;//三軸磁場和估計方向進行叉積運算,計算估計方向與三軸磁場的偏差
            eyMag = mz * wx - mx * wz;
            ezMag = mx * wy - my * wx;

            // use un-extrapolated old values between magnetometer updates
            // dubious as dT does not apply to the magnetometer calculation so
            // time scaling is embedded in KpMag and KiMag
						//使用估計的舊值與磁力計值進行更新，dT不能應用在磁力計計算中，因此時間被嵌入在KpMag 和 KiMag裡面
            gx += exMag * 5.0f;//比例增益控制磁強計收斂速度
            gy += eyMag * 5.0f;
            gz += ezMag * 5.0f;

//			eepromConfig.KiMag=0.0;
//            if (eepromConfig.KiMag > 0.0f)//用積分增益控制陀螺儀的偏差收斂速率
//            {
//                exMagInt += exMag * eepromConfig.KiMag;
//                eyMagInt += eyMag * eepromConfig.KiMag;
//                ezMagInt += ezMag * eepromConfig.KiMag;

//                gx += exMagInt;
//                gy += eyMagInt;
//                gz += ezMagInt;
//            }
        }

        //-------------------------------------------

        // integrate quaternion rate
			 //四元數微分方程，其中halfT為測量週期，g為陀螺儀角速度，其餘都是已知量，這裡使用了一階龍格庫塔法求解四元數微分方程。
        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1i = (q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2i = (q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3i = (q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0 += q0i;
        q1 += q1i;
        q2 += q2i;
        q3 += q3i;

        // normalise quaternion
				//四元數歸一化，為什麼又要歸一化呢？這是因為引入了誤差向量后四元數失去了規範性了(模不等於1了),所以要重新歸一化
        normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normR;
        q1 *= normR;
        q2 *= normR;
        q3 *= normR;

        // auxiliary variables to reduce number of repeated operations
				//把計算參考方向用到的值先都計算好,減少下面計算歐拉角時候的重複計算。
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

				//最後根據四元數方向餘弦陣和歐拉角的轉換關係，把四元數轉換成歐拉角
        margAttitude500Hz[ROLL ] = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);
        margAttitude500Hz[PITCH] = -asinf(2.0f * (q1q3 - q0q2));
        margAttitude500Hz[YAW  ] = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
        printf("margAttitude500Hz[ROLL]=%f, margAttitude500Hz[PITCH]=%f, margAttitude500Hz[YAW]=%f, \r\n",
        		margAttitude500Hz[ROLL ]*57.3,
				margAttitude500Hz[PITCH ]*57.3,
				margAttitude500Hz[YAW  ]*57.3);
    }
}
//====================================================================================================
// END OF CODE
//====================================================================================================


