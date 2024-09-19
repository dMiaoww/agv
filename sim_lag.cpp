#include "sim/steer_ddsu_lag.h"
#include "math.h"

int main(int argc,char **argv) {
    double pid_kp = atof(argv[1]);
    double pid_ki = atof(argv[2]);
    double pid_kd = atof(argv[3]);
    std::cout << pid_kp << " " << pid_ki << " " << pid_kd << std::endl;
    
    SteerDDSULag steer("1");
    static std::ofstream file;
        file.open("DDSU_PID");

    for (int i = 0; i < 400; i++)
    {
        double a = 0.5*sin(M_PI * i / 30.0);  // 输入

        static PositionPID pid(pid_kp, pid_ki, pid_kd, 0.1);
        // double ab = pid.getOutput(a, steer.real_angle, 0.5);  // 控制器输出
        steer.SetSpeed(a, 0.1);


        file << a << " " << 0.1 << " " << steer.real_angle << " "  // 实际输出
           << steer.real_vd << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    file.close();
    return 0;
}