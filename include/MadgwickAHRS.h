//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 24/06/2019   Matthieu Magnon Compute gyro bias
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    /* fusion gain */
    float beta;
    /* bias gain */
    float zeta;
    /* quaternion of sensor frame relative to auxiliary frame */
    float q0;
    float q1;
    float q2;
    float q3;
    /* euler angles from ZYX quaternion decomposition */
    float roll;
    float pitch;
    float yaw;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float fusionGain, float biasGain) { beta = fusionGain; zeta = biasGain; };
    /* 
     * dt : time interval in sec
     * gx, gy, gz : gyro in rad/s
     * ax, ay, az : accelero in g
     * mx, my, mz : magneto in gauss
     * bx, by, bz : gyro bias in rad/s
     */
    void update(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *bx, float *by, float *bz);
    void updateIMU(float dt, float gx, float gy, float gz, float ax, float ay, float az, float *bx, float *by, float *bz);
    void getAngles(float angles[3]);
    void getQuaternion(float quaternion[4]);

};
#endif

