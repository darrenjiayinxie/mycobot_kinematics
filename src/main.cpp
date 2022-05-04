#include <mycobot_kinematics/mycobot_kin.h>
#include <mycobot_kinematics/ur_kin.h>
#include <chrono>



int main(int argc, char* argv[])
{
    
    std::cout.sync_with_stdio(false);
    
    decltype(std::chrono::high_resolution_clock::now()) start;
    decltype(start) end;
    long long ticks;
    std::uniform_real_distribution<double> unif(-M_PI, M_PI);
    //Mersenne Twister: Good quality random number generator
    std::mt19937 rng; 
    //Initialize with non-deterministic seeds
    rng.seed(std::random_device{}()); 

    Eigen::Isometry3d T;
    std::vector<std::vector<double>> q_sols;
    long long total = 0;
    for (int i = 0; i < 1; i ++)
    {
        std::vector<double> q {unif(rng), unif(rng), unif(rng), unif(rng), unif(rng), unif(rng)};
     
        std::cout << "The initial value of q is:" << std::endl; 
        for (int i = 0; i < 6; i ++) {
            std::cout << q[i] << " ";
            
        }
        std::cout << std::endl;
        
        
        mycobot_kinematics::forward(q, T);
        start = std::chrono::high_resolution_clock::now();
        mycobot_kinematics::inverse(T, q_sols);
        end = std::chrono::high_resolution_clock::now();
        ticks = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "time =" <<  ticks << "[µs]" << std::endl;
        total += ticks;
    }
        
    std::cout << "average time =" <<  total/1 << "[µs]" << std::endl;
    
    
    std::cout << "The solution of T is:" << std::endl;
    for (int i = 0; i < 4; i ++) {
        printf("%1.5f %1.5f %1.5f %1.5f \n", 
        T(i,0), T(i,1), T(i,2), T(i,3));
    }

    
    std::cout << "The solutions are:" << std::endl;
    for (long unsigned int i = 0; i < q_sols.size(); i ++) {
        printf("%1.5f %1.5f %1.5f %1.5f %1.5f %1.5f\n", 
        q_sols[i][0], q_sols[i][1], q_sols[i][2], q_sols[i][3], q_sols[i][4], q_sols[i][5]);
    }
    
    
    


    return 0;
}