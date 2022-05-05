#include <mycobot_kinematics/mycobot_kin.h>


namespace mycobot_kinematics {

    const double PI = M_PI;
    const double INVALID_TAG = 100;
    const double THRESHOLD_ZERO = 1e-8;
    // modified DH parameters of mycobot (m)
    const double a2 = -0.1104; const double a3 = -0.096;
    const double d1 = 0.13156; const double d4 = 0.06462;
    const double d5 = 0.07318; const double d6 = 0.0456;
    int SIGN(double x) {
      return (x > 0) - (x < 0);
    }

    // joint limits of mycobot 
    //const std::vector<double> l_joint = {0, PI*-165/180, PI*-165/180, PI*-165/180, PI*-165/180, PI*-165/180, PI*-175/180};
    //const std::vector<double> u_joint = {0, PI*165/180,  PI*165/180,  PI*165/180,  PI*165/180,  PI*165/180,  PI*175/180};


    void wrapTo2Pi(double& angle){
        angle = fmod(angle, 2*PI);
        if (angle < 0)
            angle += 2*PI;

         // round up the boundary
        if (fabs(angle - 2*PI) < THRESHOLD_ZERO) {
            angle = - 2*PI;
        }

        if (fabs(angle) < THRESHOLD_ZERO) {
            angle = 0;
        }
        
    }
    void wrapToPi(double& angle) {

        angle = fmod(angle + PI, 2*PI);
        if (angle < 0)
            angle += 2*PI;
        angle -= PI;

         // round up the boundary
        if (fabs(angle - PI) < THRESHOLD_ZERO) {
            angle = - PI;
        }

        if (fabs(angle) < THRESHOLD_ZERO) {
            angle = 0;
        }

        // check valid range of joint i
        //if (angle > u_joint[i] || angle < l_joint[i]) {
        //    angle = INVALID_TAG;
        //}
    }
    
    
       
    
    void forward(const std::vector<double>& q, Eigen::Isometry3d& T) {
        
        std::vector<double>::const_iterator it = q.begin();
        double s1 = sin(*it), s2 = sin(*(it +1)), s5 = sin(*(it +4)), s6 = sin(*(it +5));
        double c1 = cos(*it), c2 = cos(*(it +1)), c5 = cos(*(it +4)), c6 = cos(*(it +5));
        double c23 = cos(*(it +1) + *(it +2)), s23 = sin(*(it +1) + *(it +2)), 
               c234 = cos(*(it +1) + *(it +2) + *(it +3)), s234 = sin(*(it +1) + *(it +2) + *(it +3));
             
        
        T(0,0) =  c6*s1*s5 - c1*s6*s234 + c1*c5*c6*c234; 
        T(0,1) = -c1*c6*s234 - s1*s5*s6 - c1*c5*c234*s6;
        T(0,2) =  c5*s1 - c1*c234*s5;               
        T(0,3) =  d4*s1 + c1*(a2*c2 + a3*c23) + c5*d6*s1 + c1*d5*s234 - c1*c234*d6*s5;

        T(1,0) =  c5*c6*c234*s1 - s1*s6*s234 - c1*c6*s5; 
        T(1,1) =  c1*s5*s6 - c6*s1*s234 - c5*c234*s1*s6;
        T(1,2) = -c1*c5 - c234*s1*s5;
        T(1,3) =  s1*(a2*c2 + a3*c23) - c1*d4 - c1*c5*d6 + d5*s1*s234 - c234*d6*s1*s5;

        T(2,0) = c234*s6 + c5*c6*s234; 
        T(2,1) = c6*c234 - c5*s6*s234; 
        T(2,2) = -s5*s234; 
        T(2,3) = d1 - c234*d5 + a2*s2 + a3*s23 - d6*s5*s234;
        
        T(3,0) = 0; T(3,1) = 0; T(3,2) = 0; T(3,3) = 1;        
    }

    
    // the implementation of the function follows the tutorial http://rasmusan.blog.aau.dk/files/ur5_kinematics.pdf
    std::vector<double> cal_angle(const Eigen::Isometry3d& T, const std::vector<double>& candidate, const int step) {
        std::vector<double> theta(2, INVALID_TAG);
        switch (step) {
        /////////////////////// find sholder rotated joint theta1 (from eq. 9) /////////////////////////////
            case 0:
            {   
            
                double p05x = -T(0,2)*d6 + T(0,3),  
                       p05y = -T(1,2)*d6 + T(1,3);
                double p05xy = sqrt(p05x*p05x + p05y*p05y);
                
                double term;
                // edge case
                if (fabs(p05xy) < THRESHOLD_ZERO) {
                    term = -SIGN(d4)*SIGN(p05xy);
                }else {
                    term = d4/p05xy;
                }
                
                if (fabs(term) - 1 > THRESHOLD_ZERO) {
                    // no solution exists
                    return theta;
                }

                /*compute theta: 
                    (1) acos on boundary of 1
                    (2) acos on boundary of -1
                    (3) acos within the range
                */
               
                if (fabs(term - 1) < THRESHOLD_ZERO) {
                    theta[0] = atan2(p05y, p05x) + acos(1) + PI/2;
                    theta[1] = atan2(p05y, p05x) - acos(1) + PI/2;
                }else if (fabs(term + 1) < THRESHOLD_ZERO) {
                    theta[0] = atan2(p05y, p05x) + acos(-1) + PI/2;
                    theta[1] = atan2(p05y, p05x) - acos(-1) + PI/2;
                }else {
                    theta[0] = atan2(p05y, p05x) + acos(term) + PI/2;
                    theta[1] = atan2(p05y, p05x) - acos(term) + PI/2;
                }

                // wrap theta to [-PI, PI] and validate its range
                wrapTo2Pi(theta[0]);
                wrapTo2Pi(theta[1]);

                // check whether two solution are same
                if (fabs(theta[0] - theta[1]) < THRESHOLD_ZERO) {
                    theta[1] = INVALID_TAG;
                }

                break;

            }
        /////////////////////// find wrist 2 joint theta5 (from eq. 12) /////////////////////////////
            case 1:
            {   
                // compute the input
                double numer = (T(0,3)*sin(candidate[0]) - T(1,3)*cos(candidate[0]) -d4);
                double term;
                // edge cases
                
                if(fabs(fabs(numer) - fabs(d6)) < THRESHOLD_ZERO)
                    term = SIGN(numer) * SIGN(d6);
                else
                    term = numer / d6;
                /*compute theta: 
                    (1) acos on boundary of 1
                    (2) acos on boundary of -1
                    (3) acos within the range
                */
                if (fabs(term - 1) < THRESHOLD_ZERO) {
                    theta[0] = acos(1);
                    theta[1] = -acos(1);
                }else if (fabs(term + 1) < THRESHOLD_ZERO) {
                    theta[0] = acos(-1);
                    theta[1] = -acos(-1);
                }else {
                    theta[0] = acos(term);
                    theta[1] = -acos(term);
                }
                        
                
                // wrap theta to [-0.5*PI, 1.5*PI) and validate its range
                wrapTo2Pi(theta[0]);
                wrapTo2Pi(theta[1]);
                
                // check whether two solution are same
                if (fabs(theta[0] - theta[1]) < THRESHOLD_ZERO) {
                    theta[1] = INVALID_TAG;
                }
                break;

            }
        ////////////////////////////// find wrist 3 joint theta6 (from eq. 16) //////////////////////////////
            case 2:
            {
                // compute the input
                double term1 = (-T(0,1)*sin(candidate[0]) + T(1,1)*cos(candidate[0]));
                double term2 = ( T(0,0)*sin(candidate[0]) - T(1,0)*cos(candidate[0]));

                // edge cases
                // in this case theta6 is redundant to be an arbitary value
                if (fabs(sin(candidate[1])) < THRESHOLD_ZERO) {

                    std::uniform_real_distribution<double> unif(-M_PI, M_PI);
                    //Mersenne Twister: Good quality random number generator
                    std::mt19937 rng; 
                    //Initialize with non-deterministic seeds
                    rng.seed(std::random_device{}()); 

                    theta[0] = unif(rng);
                    
                    return theta;
                }


                // compute theta
                theta[0] = atan2(term1*SIGN(sin(candidate[1])), term2*SIGN(sin(candidate[1])));
                

                // wrap theta to [-PI, PI) and validate its range
                wrapTo2Pi(theta[0]);

                break;

            }
        ///////////////////////////// find R joint theta3 (from eq. 19) ////////////////////////////
            case 3:
            {
                // compute the input
                double s1 = sin(candidate[0]), s6 = sin(candidate[2]);
                double c1 = cos(candidate[0]), c6 = cos(candidate[2]);


                double p14x = c1*T(0,3) + T(1,3)*s1 - c1*d6*T(0,2) - d6*T(1,2)*s1 
                              + d5*T(1,0)*s1*s6 + c1*c6*d5*T(0,1) + c1*d5*T(0,0)*s6 + c6*d5*T(1,1)*s1;
                double p14z = T(2,3) - d1 - d6*T(2,2) + c6*d5*T(2,1) + d5*T(2,0)*s6;

                double term = (p14x*p14x + p14z*p14z - a2*a2 - a3*a3)/(2*a2*a3);

                // edge case
                if (fabs(term) - 1 > THRESHOLD_ZERO) {
                    return theta;
                }

                /*compute theta: 
                    (1) acos on boundary of 1
                    (2) acos on boundary of -1
                    (3) acos within the range
                */
                if (fabs(term - 1) < THRESHOLD_ZERO) {
                    theta[0] = acos(1);
                    theta[1] = -acos(1);
                }else if (fabs(term + 1) < THRESHOLD_ZERO) {
                    theta[0] = acos(-1);
                    theta[1] = -acos(-1);
                }else {
                    theta[0] = acos(term);
                    theta[1] = -acos(term);
                }

                // wrap theta to [-PI, PI) and validate its range
                wrapTo2Pi(theta[0]);
                wrapTo2Pi(theta[1]);

                // check whether two solution are same
                if (fabs(theta[0] - theta[1]) < THRESHOLD_ZERO) {
                    theta[1] = INVALID_TAG;
                }

                break;

            }
        ///////////////////////////// find R joint theta2 (from eq. 22) ////////////////////////////
            case 4:
            {
                // compute the input
                double s1 = sin(candidate[0]), s6 = sin(candidate[2]);
                double c1 = cos(candidate[0]), c6 = cos(candidate[2]);
                

                double p14x = c1*T(0,3) + T(1,3)*s1 - c1*d6*T(0,2) - d6*T(1,2)*s1 
                              + d5*T(1,0)*s1*s6 + c1*c6*d5*T(0,1) + c1*d5*T(0,0)*s6 + c6*d5*T(1,1)*s1;
                double p14z = T(2,3) - d1 - d6*T(2,2) + c6*d5*T(2,1) + d5*T(2,0)*s6;

                double term = (-a3*sin(candidate[3])/sqrt(p14x*p14x + p14z*p14z));

                // edge case
                if (fabs(term) - 1 > THRESHOLD_ZERO) {
                    return theta;
                }

                /*compute theta: 
                    (1) asin on boundary of 1
                    (2) asin on boundary of -1
                    (3) asin within the range
                */
                if (fabs(term - 1) < THRESHOLD_ZERO) {
                    theta[0] = atan2(-p14z, -p14x) - asin(1);
                }else if (fabs(term + 1) < THRESHOLD_ZERO) {
                    theta[0] = atan2(-p14z, -p14x) - asin(-1);
                }else {
                    theta[0] = atan2(-p14z, -p14x) - asin(term);
                }


                // wrap theta to [-1.5*PI, 0.5*PI) and validate its range
                wrapTo2Pi(theta[0]);

                break;
            }
        ///////////////////////////// find R joint theta4 (from eq. 23) ////////////////////////////
            case 5:
            {
                // compute the input
                

                double s1 = sin(candidate[0]), s5 = sin(candidate[1]), s6 = sin(candidate[2]);
                double c1 = cos(candidate[0]), c5 = cos(candidate[1]), c6 = cos(candidate[2]);
                double c23 = cos(candidate[4] + candidate[3]), s23 = sin(candidate[4] + candidate[3]);

                double X34y = s5*(c1*T(0,2)*s23 - c23*T(2,2) + T(1,2)*s1*s23) 
                            + c5*s6*(c1*T(0,1)*s23 - c23*T(2,1) + T(1,1)*s1*s23) 
                            - c5*c6*(c1*T(0,0)*s23 - c23*T(2,0) + T(1,0)*s1*s23);
                double X34x = c5*c6*(T(2,0)*s23 + c1*c23*T(0,0) + c23*T(1,0)*s1) 
                            - s5*(T(2,2)*s23 + c1*c23*T(0,2) + c23*T(1,2)*s1) 
                            - c5*s6*(T(2,1)*s23 + c1*c23*T(0,1) + c23*T(1,1)*s1);

                // compute theta
                theta[0] = atan2(X34y, X34x);

                // wrap theta to [-1.5*PI, 0.5PI) and validate its range
                wrapTo2Pi(theta[0]);

                break;
            }
        }
        return theta;
    }

    void solutions_tree(const Eigen::Isometry3d& T, int step, std::vector<double>& candidate, std::vector<std::vector<double>>& solutions) {

        // when step == 6, add candidate to solutions and return
        if (step == 6) {
            //theta 1
            wrapToPi(candidate[0]);
            //theta 2
            candidate[4] += PI/2;
            wrapToPi(candidate[4]);
            //theta 3
            wrapToPi(candidate[3]);
            //theta 4
            candidate[5] += PI/2;
            wrapToPi(candidate[5]);
            //theta5
            candidate[1] -= PI/2;
            wrapToPi(candidate[1]);
            //theta6
            wrapToPi(candidate[2]);
            
            solutions.push_back({candidate[0], candidate[4], candidate[3],
                                 candidate[5], candidate[1], candidate[2]});
            return;
        }
        
        //std::cout << "With candidate: ";
        //for (int i = 0; i < candidate.size(); i ++) {
        //    std::cout << candidate[i] << " ";
        //}
        //std::cout << std::endl;
        
        std::vector<double> theta = cal_angle(T, candidate, step);
        

        // In step 0, 1 and 3, there exists two possible solutions with given candidate
        if (step == 0 || step == 1 || step == 3) {
            if (theta[0] != INVALID_TAG) {
                candidate[step] = theta[0];
                solutions_tree(T, step + 1, candidate, solutions);
                candidate[step] = INVALID_TAG;
            }
            
            if (theta[1] != INVALID_TAG) {
                candidate[step] = theta[1];
                solutions_tree(T, step + 1, candidate, solutions);
                candidate[step] = INVALID_TAG;
            }
        
        // In step 2, 4 and 5, there exists one possible solution with given candidate
        }else {
            if (theta[0] != INVALID_TAG) {
                candidate[step] = theta[0];
                solutions_tree(T, step + 1, candidate, solutions);
                candidate[step] = INVALID_TAG;
            }
        }
    }
    
    void inverse(const Eigen::Isometry3d& T, std::vector<std::vector<double>>& q_sols) {
        
        
        std::vector<double> candidate(8, INVALID_TAG); 

        solutions_tree(T, 0, candidate, q_sols);


    }
} // namespace mycobot_kinematics


