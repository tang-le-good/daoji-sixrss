#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

class XianDuojiCoordidateModel
{
public:
    XianDuojiCoordidateModel()
    {
        // 创建一个ROS节点句柄
        ros::NodeHandle nh;

        // Initialize the PCL visualizer
        viewer->initCameraParameters();
        viewer->setBackgroundColor(0.1, 0.1, 0.1);
        // 添加这一行以避免线程问题
        viewer->setShowFPS(false);

        // 将A_is绕z轴旋转30度


        ImageShow_f();
    }
    //绕Y轴旋转矩阵运算
    pcl::PointXYZ R_y(double rotate_angle, pcl::PointXYZ A, const double pi)
    {
        // 将点A绕Z轴旋转rotate_angle度（角度转弧度）
        double angle_rad = rotate_angle * pi / 180.0;
        Eigen::Vector3f point(A.x, A.y, A.z);
        Eigen::Matrix3f rotation;
        rotation << cos(angle_rad), 0, sin(angle_rad),
                     0, 1, 0,
                     -sin(angle_rad), 0, cos(angle_rad);
        Eigen::Vector3f rotated = rotation * point;
        return pcl::PointXYZ(rotated.x(), rotated.y(), rotated.z());
    }
    //绕Z轴旋转矩阵运算
    pcl::PointXYZ R_z(double rotate_angle, pcl::PointXYZ A, const double pi)
    {
        // 将点A绕Z轴旋转rotate_angle度（角度转弧度）
        double angle_rad = rotate_angle * pi / 180.0;
        Eigen::Vector3f point(A.x, A.y, A.z);
        Eigen::Matrix3f rotation;
        rotation << cos(angle_rad), -sin(angle_rad), 0,
                sin(angle_rad),  cos(angle_rad), 0,
                0,              0,              1;
        Eigen::Vector3f rotated = rotation * point;
        return pcl::PointXYZ(rotated.x(), rotated.y(), rotated.z());
    }

    // 旋转平移矩阵运算
    pcl::PointXYZ RP_xyz(double rotatex_angle, double rotatey_angle, double rotatez_angle, 
                         double x, double y, double z, 
                         pcl::PointXYZ A, const double pi)
    {
        // 将点A绕Z轴旋转rotate_angle度（角度转弧度）
        double anglez_rad = rotatez_angle * pi / 180.0;
        double angley_rad = rotatey_angle * pi / 180.0;
        double anglex_rad = rotatex_angle * pi / 180.0;
        Eigen::Vector3f point(A.x, A.y, A.z);
        Eigen::Vector3f P(x, y, z);
        Eigen::Matrix3f rotationz, rotationy, rotationx;
        rotationz << cos(anglez_rad), -sin(anglez_rad), 0,
                     sin(anglez_rad),  cos(anglez_rad), 0,
                     0,              0,              1;

        rotationy << cos(angley_rad), 0, sin(angley_rad),
                     0, 1, 0,
                     -sin(angley_rad), 0, cos(angley_rad);
        
        rotationx << 1, 0, 0,
                     0, cos(anglex_rad), -sin(anglex_rad),
                     0, sin(anglex_rad), cos(anglex_rad);

        Eigen::Vector3f rotated = rotationz * rotationy * rotationx * point + P;
        return pcl::PointXYZ(rotated.x(), rotated.y(), rotated.z());
    }
    // 两个向量的点乘运算
    double PointsDot(pcl::PointXYZ A, pcl::PointXYZ B) 
    {
        Eigen::Vector3f A_t(A.x, A.y, A.z);  // 将点A转换为Eigen向量
        Eigen::Vector3f B_t(B.x, B.y, B.z);  // 将点B转换为Eigen向量
        float dot_product = A_t.dot(B_t);     // 调用Eigen的dot()函数计算点积
        return dot_product;                   // 返回标量结果
    }

    // 计算向量模长
    double VectorMagnitude(pcl::PointXYZ A) {
        Eigen::Vector3f A_t(A.x, A.y, A.z);  // 将点转换为Eigen向量
        return A_t.norm();                   // 调用Eigen的norm()函数计算模长
    }

    //两个点相减运算
    pcl::PointXYZ PointsSub(pcl::PointXYZ A, pcl::PointXYZ B)
    {
        Eigen::Vector3f A_t(A.x, A.y, A.z);
        Eigen::Vector3f B_t(B.x, B.y, B.z);
        Eigen::Vector3f sum;
        sum = A_t - B_t;
        return pcl::PointXYZ(sum.x(), sum.y(), sum.z());
    }
    //两个点相加运算
    pcl::PointXYZ PointsAdd(pcl::PointXYZ A, pcl::PointXYZ B)
    {
        Eigen::Vector3f A_t(A.x, A.y, A.z);
        Eigen::Vector3f B_t(B.x, B.y, B.z);
        Eigen::Vector3f sum;
        sum = A_t + B_t;
        return pcl::PointXYZ(sum.x(), sum.y(), sum.z());
    }

    // 计算约束角
    double ComputeConstraintAngular(pcl::PointXYZ A, pcl::PointXYZ B)
    {
        double zeta_fenzi_1 = PointsDot(A, B);
        double A_magn = VectorMagnitude(A);
        double B_magn = VectorMagnitude(B);
        double cos_zeta_1 = zeta_fenzi_1 / (A_magn * B_magn);
        double zeta = std::acos(cos_zeta_1) / PI * 180.0;
        return zeta;
    }

    void ImageShow_f()
    {
        while(ros::ok() )
        {
            sleep(0.03); // 延时0.1秒

            ros::param::get("/xian_stewart_control_params_server/input_alpha", input_alpha);
            ros::param::get("/xian_stewart_control_params_server/input_beta", input_beta);
            ros::param::get("/xian_stewart_control_params_server/input_gamma", input_gamma);
            ros::param::get("/xian_stewart_control_params_server/input_x", input_x);
            ros::param::get("/xian_stewart_control_params_server/input_y", input_y);
            ros::param::get("/xian_stewart_control_params_server/input_z", input_z);

            // 求A_i
            A_1 = R_z(theta_1, A_is, PI);
            A_2 = R_z(theta_2, A_is, PI);
            A_3 = R_z(theta_3, A_is, PI);
            A_4 = R_z(theta_4, A_is, PI);
            A_5 = R_z(theta_5, A_is, PI);
            A_6 = R_z(theta_6, A_is, PI);

            // 求C_i
            C_1 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, C_1m, PI);
            C_2 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, C_2m, PI);
            C_3 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, C_3m, PI);
            C_4 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, C_4m, PI);
            C_5 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, C_5m, PI);
            C_6 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, C_6m, PI);
            camera_center = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, camera_center_m, PI);

            // 求C_iA_i向量
            C_1A_1 = PointsSub(A_1, C_1);
            C_2A_2 = PointsSub(A_2, C_2);
            C_3A_3 = PointsSub(A_3, C_3);
            C_4A_4 = PointsSub(A_4, C_4);
            C_5A_5 = PointsSub(A_5, C_5);
            C_6A_6 = PointsSub(A_6, C_6);

            // 求omega_i
            ac_11 = C_1A_1.x;
            ac_21 = C_2A_2.x;
            ac_31 = C_3A_3.x;
            ac_41 = C_4A_4.x;
            ac_51 = C_5A_5.x;
            ac_61 = C_6A_6.x;

            ac_12 = C_1A_1.y;
            ac_22 = C_2A_2.y;
            ac_32 = C_3A_3.y;
            ac_42 = C_4A_4.y;
            ac_52 = C_5A_5.y;
            ac_62 = C_6A_6.y;

            ac_13 = C_1A_1.z;
            ac_23 = C_2A_2.z;
            ac_33 = C_3A_3.z;
            ac_43 = C_4A_4.z;
            ac_53 = C_5A_5.z;
            ac_63 = C_6A_6.z;

            M1_11 = cos(theta_1 * PI / 180.0) * cos(phi_1 * PI / 180.0) - sin(theta_1 * PI / 180.0) * sin(phi_1 * PI / 180.0);
            M2_11 = cos(theta_2 * PI / 180.0) * cos(phi_2 * PI / 180.0) - sin(theta_2 * PI / 180.0) * sin(phi_2 * PI / 180.0);
            M3_11 = cos(theta_3 * PI / 180.0) * cos(phi_3 * PI / 180.0) - sin(theta_3 * PI / 180.0) * sin(phi_3 * PI / 180.0);
            M4_11 = cos(theta_4 * PI / 180.0) * cos(phi_4 * PI / 180.0) - sin(theta_4 * PI / 180.0) * sin(phi_4 * PI / 180.0);
            M5_11 = cos(theta_5 * PI / 180.0) * cos(phi_5 * PI / 180.0) - sin(theta_5 * PI / 180.0) * sin(phi_5 * PI / 180.0);
            M6_11 = cos(theta_6 * PI / 180.0) * cos(phi_6 * PI / 180.0) - sin(theta_6 * PI / 180.0) * sin(phi_6 * PI / 180.0);

            M1_21 = sin(theta_1 * PI / 180.0) * cos(phi_1 * PI / 180.0) + cos(theta_1 * PI / 180.0) * sin(phi_1 * PI / 180.0);
            M2_21 = sin(theta_2 * PI / 180.0) * cos(phi_2 * PI / 180.0) + cos(theta_2 * PI / 180.0) * sin(phi_2 * PI / 180.0);
            M3_21 = sin(theta_3 * PI / 180.0) * cos(phi_3 * PI / 180.0) + cos(theta_3 * PI / 180.0) * sin(phi_3 * PI / 180.0);
            M4_21 = sin(theta_4 * PI / 180.0) * cos(phi_4 * PI / 180.0) + cos(theta_4 * PI / 180.0) * sin(phi_4 * PI / 180.0);
            M5_21 = sin(theta_5 * PI / 180.0) * cos(phi_5 * PI / 180.0) + cos(theta_5 * PI / 180.0) * sin(phi_5 * PI / 180.0);
            M6_21 = sin(theta_6 * PI / 180.0) * cos(phi_6 * PI / 180.0) + cos(theta_6 * PI / 180.0) * sin(phi_6 * PI / 180.0);

            a_1 = -2 * LA * ac_13;
            a_2 = -2 * LA * ac_23;
            a_3 = -2 * LA * ac_33;
            a_4 = -2 * LA * ac_43;
            a_5 = -2 * LA * ac_53;
            a_6 = -2 * LA * ac_63;

            b_1 = 2 * LA * M1_11 * ac_11 + 2 * LA * M1_21 * ac_12;
            b_2 = 2 * LA * M2_11 * ac_21 + 2 * LA * M2_21 * ac_22;
            b_3 = 2 * LA * M3_11 * ac_31 + 2 * LA * M3_21 * ac_32;
            b_4 = 2 * LA * M4_11 * ac_41 + 2 * LA * M4_21 * ac_42;
            b_5 = 2 * LA * M5_11 * ac_51 + 2 * LA * M5_21 * ac_52;
            b_6 = 2 * LA * M6_11 * ac_61 + 2 * LA * M6_21 * ac_62;

            LCs_1 = C_1A_1.x * C_1A_1.x + C_1A_1.y * C_1A_1.y + C_1A_1.z * C_1A_1.z;
            LCs_2 = C_2A_2.x * C_2A_2.x + C_2A_2.y * C_2A_2.y + C_2A_2.z * C_2A_2.z;
            LCs_3 = C_3A_3.x * C_3A_3.x + C_3A_3.y * C_3A_3.y + C_3A_3.z * C_3A_3.z;
            LCs_4 = C_4A_4.x * C_4A_4.x + C_4A_4.y * C_4A_4.y + C_4A_4.z * C_4A_4.z;
            LCs_5 = C_5A_5.x * C_5A_5.x + C_5A_5.y * C_5A_5.y + C_5A_5.z * C_5A_5.z;
            LCs_6 = C_6A_6.x * C_6A_6.x + C_6A_6.y * C_6A_6.y + C_6A_6.z * C_6A_6.z;

            c_1 = LA*LA -LB*LB + LCs_1;
            c_2 = LA*LA -LB*LB + LCs_2;
            c_3 = LA*LA -LB*LB + LCs_3;
            c_4 = LA*LA -LB*LB + LCs_4;
            c_5 = LA*LA -LB*LB + LCs_5;
            c_6 = LA*LA -LB*LB + LCs_6;

            omega_1 = (std::asin(c_1/std::sqrt(a_1*a_1+b_1*b_1)) - std::atan(b_1/a_1)) / PI * 180.0;
            omega_2 = (std::asin(c_2/std::sqrt(a_2*a_2+b_2*b_2)) - std::atan(b_2/a_2)) / PI * 180.0;
            omega_3 = (std::asin(c_3/std::sqrt(a_3*a_3+b_3*b_3)) - std::atan(b_3/a_3)) / PI * 180.0;
            omega_4 = (std::asin(c_4/std::sqrt(a_4*a_4+b_4*b_4)) - std::atan(b_4/a_4)) / PI * 180.0;
            omega_5 = (std::asin(c_5/std::sqrt(a_5*a_5+b_5*b_5)) - std::atan(b_5/a_5)) / PI * 180.0;
            omega_6 = (std::asin(c_6/std::sqrt(a_6*a_6+b_6*b_6)) - std::atan(b_6/a_6)) / PI * 180.0;        

            // 动平台上的铰链约束角计算
            // E点从动平台坐标系转化到静平台坐标系
            E_1 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, E_1m, PI);
            E_2 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, E_2m, PI);
            E_3 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, E_3m, PI);
            E_4 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, E_4m, PI);
            E_5 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, E_5m, PI);
            E_6 = RP_xyz(input_alpha, input_beta, input_gamma, input_x, input_y, input_z, E_6m, PI);

            // 求向量C_iE_i
            C_1E_1_vec = PointsSub(E_1, C_1);
            C_2E_2_vec = PointsSub(E_2, C_2);
            C_3E_3_vec = PointsSub(E_3, C_3);
            C_4E_4_vec = PointsSub(E_4, C_4);
            C_5E_5_vec = PointsSub(E_5, C_5);
            C_6E_6_vec = PointsSub(E_6, C_6);
            // 求向量C_iB_i   
            C_1B_1_vec = PointsSub(B_1, C_1);
            C_2B_2_vec = PointsSub(B_2, C_2);
            C_3B_3_vec = PointsSub(B_3, C_3);
            C_4B_4_vec = PointsSub(B_4, C_4);
            C_5B_5_vec = PointsSub(B_5, C_5);
            C_6B_6_vec = PointsSub(B_6, C_6);

            //计算铰链座轴线与连杆夹角
            zeta_1 = ComputeConstraintAngular(C_1E_1_vec, C_1B_1_vec);
            zeta_2 = ComputeConstraintAngular(C_2E_2_vec, C_2B_2_vec);
            zeta_3 = ComputeConstraintAngular(C_3E_3_vec, C_3B_3_vec);
            zeta_4 = ComputeConstraintAngular(C_4E_4_vec, C_4B_4_vec);
            zeta_5 = ComputeConstraintAngular(C_5E_5_vec, C_5B_5_vec);
            zeta_6 = ComputeConstraintAngular(C_6E_6_vec, C_6B_6_vec);           
            
            // 求B点，
            // 求B_id
            B_1d = R_y(omega_1, P_d, PI);
            B_2d = R_y(omega_2, P_d, PI);
            B_3d = R_y(omega_3, P_d, PI);
            B_4d = R_y(omega_4, P_d, PI);
            B_5d = R_y(omega_5, P_d, PI);
            B_6d = R_y(omega_6, P_d, PI);

            // 求B_is
            B_1s = PointsAdd(R_z(phi_1, B_1d, PI), A_is);
            B_2s = PointsAdd(R_z(phi_2, B_2d, PI), A_is);
            B_3s = PointsAdd(R_z(phi_3, B_3d, PI), A_is);
            B_4s = PointsAdd(R_z(phi_4, B_4d, PI), A_is);
            B_5s = PointsAdd(R_z(phi_5, B_5d, PI), A_is);
            B_6s = PointsAdd(R_z(phi_6, B_6d, PI), A_is);

            // 求B_i
            B_1 = R_z(theta_1, B_1s, PI);
            B_2 = R_z(theta_2, B_2s, PI);
            B_3 = R_z(theta_3, B_3s, PI);
            B_4 = R_z(theta_4, B_4s, PI);
            B_5 = R_z(theta_5, B_5s, PI);
            B_6 = R_z(theta_6, B_6s, PI);
            
            //舵机上的铰链约束角计算
            D_1d = R_y(omega_1, TD_id135, PI);
            D_2d = R_y(omega_2, TD_id246, PI);
            D_3d = R_y(omega_3, TD_id135, PI);
            D_4d = R_y(omega_4, TD_id246, PI);
            D_5d = R_y(omega_5, TD_id135, PI);
            D_6d = R_y(omega_6, TD_id246, PI);

            D_1s = PointsAdd(R_z(phi_1, D_1d, PI), A_is);
            D_2s = PointsAdd(R_z(phi_2, D_2d, PI), A_is);
            D_3s = PointsAdd(R_z(phi_3, D_3d, PI), A_is);
            D_4s = PointsAdd(R_z(phi_4, D_4d, PI), A_is);
            D_5s = PointsAdd(R_z(phi_5, D_5d, PI), A_is);
            D_6s = PointsAdd(R_z(phi_6, D_6d, PI), A_is);

            D_1 = R_z(theta_1, D_1s, PI);
            D_2 = R_z(theta_2, D_2s, PI);
            D_3 = R_z(theta_3, D_3s, PI);
            D_4 = R_z(theta_4, D_4s, PI);
            D_5 = R_z(theta_5, D_5s, PI);
            D_6 = R_z(theta_6, D_6s, PI);
            
            // 求向量B_iD_i
            B_1D_1_vec = PointsSub(D_1, B_1);
            B_2D_2_vec = PointsSub(D_2, B_2);
            B_3D_3_vec = PointsSub(D_3, B_3);
            B_4D_4_vec = PointsSub(D_4, B_4);
            B_5D_5_vec = PointsSub(D_5, B_5);
            B_6D_6_vec = PointsSub(D_6, B_6);
            // 求向量B_iC_i   
            B_1C_1_vec = PointsSub(C_1, B_1);
            B_2C_2_vec = PointsSub(C_2, B_2);
            B_3C_3_vec = PointsSub(C_3, B_3);
            B_4C_4_vec = PointsSub(C_4, B_4);
            B_5C_5_vec = PointsSub(C_5, B_5);
            B_6C_6_vec = PointsSub(C_6, B_6);

            delta_1 = 90.0 - ComputeConstraintAngular(B_1D_1_vec, B_1C_1_vec);
            delta_2 = 90.0 - ComputeConstraintAngular(B_2D_2_vec, B_2C_2_vec);
            delta_3 = 90.0 - ComputeConstraintAngular(B_3D_3_vec, B_3C_3_vec);
            delta_4 = 90.0 - ComputeConstraintAngular(B_4D_4_vec, B_4C_4_vec);
            delta_5 = 90.0 - ComputeConstraintAngular(B_5D_5_vec, B_5C_5_vec);
            delta_6 = 90.0 - ComputeConstraintAngular(B_6D_6_vec, B_6C_6_vec);

            
            if (std::isnan(omega_1) || std::isnan(omega_2) || std::isnan(omega_3) || std::isnan(omega_4) || std::isnan(omega_5) || std::isnan(omega_6) ||
                std::isnan(delta_1) || std::isnan(delta_2) || std::isnan(delta_3) || std::isnan(delta_4) || std::isnan(delta_5) || std::isnan(delta_6) ||
                std::isnan(zeta_1) || std::isnan(zeta_2) || std::isnan(zeta_3) || std::isnan(zeta_4) || std::isnan(zeta_5) || std::isnan(zeta_6) )
            {
                printf("运动学反解无效！\n");
                
                
                
            }
            else
            {
                if(delta_1 > constraint_angle_delta || delta_2 > constraint_angle_delta || delta_3 > constraint_angle_delta || delta_4 > constraint_angle_delta || delta_5 > constraint_angle_delta || delta_6 > constraint_angle_delta ||
                 delta_1 < -constraint_angle_delta || delta_2 < -constraint_angle_delta || delta_3 < -constraint_angle_delta || delta_4 < -constraint_angle_delta || delta_5 < -constraint_angle_delta || delta_6 < -constraint_angle_delta ||
                  zeta_1 > constraint_angle_zata || zeta_2 > constraint_angle_zata || zeta_3 > constraint_angle_zata || zeta_4 > constraint_angle_zata || zeta_5 > constraint_angle_zata || zeta_6 > constraint_angle_zata ||
                  zeta_1 < -constraint_angle_zata || zeta_2 < -constraint_angle_zata || zeta_3 < -constraint_angle_zata || zeta_4 < -constraint_angle_zata || zeta_5 < -constraint_angle_zata || zeta_6 < -constraint_angle_zata)
                {
                   printf("铰链约束角超过限位\n");
                }
                else
                {
                   ros::param::set("/xian_stewart_control_params_server/xian_arm1_cmd", omega_1);
                   ros::param::set("/xian_stewart_control_params_server/xian_arm2_cmd", omega_2);
                   ros::param::set("/xian_stewart_control_params_server/xian_arm3_cmd", omega_3);
                   ros::param::set("/xian_stewart_control_params_server/xian_arm4_cmd", omega_4);
                   ros::param::set("/xian_stewart_control_params_server/xian_arm5_cmd", omega_5);
                   ros::param::set("/xian_stewart_control_params_server/xian_arm6_cmd", omega_6);
                   printf("omega_1: %.02f omega_2: %.02f omega_3: %.02f omega_4: %.02f omega_5: %.02f omega_6: %.02f \n", 
                           omega_1, omega_2, omega_3 ,omega_4 ,omega_5 ,omega_6);
                   printf("delta_1: %.02f delta_2: %.02f delta_3: %.02f delta_4: %.02f delta_5: %.02f delta_6: %.02f \n", 
                       delta_1, delta_2, delta_3 ,delta_4 ,delta_5 ,delta_6);
                   printf("zeta_1: %.02f zeta_2: %.02f zeta_3: %.02f zeta_4: %.02f zeta_5: %.02f zeta_6: %.02f \n", 
                       zeta_1, zeta_2, zeta_3 ,zeta_4 ,zeta_5 ,zeta_6);
                }
                // ros::param::set("/xian_stewart_control_params_server/xian_arm1_cmd", omega_1);
                // ros::param::set("/xian_stewart_control_params_server/xian_arm2_cmd", omega_2);
                // ros::param::set("/xian_stewart_control_params_server/xian_arm3_cmd", omega_3);
                // ros::param::set("/xian_stewart_control_params_server/xian_arm4_cmd", omega_4);
                // ros::param::set("/xian_stewart_control_params_server/xian_arm5_cmd", omega_5);
                // ros::param::set("/xian_stewart_control_params_server/xian_arm6_cmd", omega_6);
                // printf("omega_1: %.02f omega_2: %.02f omega_3: %.02f omega_4: %.02f omega_5: %.02f omega_6: %.02f \n", 
                //         omega_1, omega_2, omega_3 ,omega_4 ,omega_5 ,omega_6);
                // printf("delta_1: %.02f delta_2: %.02f delta_3: %.02f delta_4: %.02f delta_5: %.02f delta_6: %.02f \n", 
                //     delta_1, delta_2, delta_3 ,delta_4 ,delta_5 ,delta_6);
                // printf("zeta_1: %.02f zeta_2: %.02f zeta_3: %.02f zeta_4: %.02f zeta_5: %.02f zeta_6: %.02f \n", 
                //     zeta_1, zeta_2, zeta_3 ,zeta_4 ,zeta_5 ,zeta_6);
                
            }
            

            

            
            



            if (!viewer->wasStopped())
            {
                // Clear previous viewer contents
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();


                // 绘制坐标轴
                viewer->addLine(O, X, 1.0, 0.0, 0.0, "X_axis"); // 红色
                viewer->addLine(O, Y, 0.0, 1.0, 0.0, "Y_axis"); // 绿色
                viewer->addLine(O, Z, 0.0, 0.0, 1.0, "Z_axis"); // 蓝色
                
                // 绘制A_1到A_6
                viewer->addSphere(A_1, point_size, 1.0, 0.0, 0.0, "A_1"); // 红色    
                viewer->addSphere(A_2, point_size, 1.0, 1.0, 0.0, "A_2"); // 黄色
                viewer->addSphere(A_3, point_size, 0.0, 0.0, 1.0, "A_3"); // 蓝色
                viewer->addSphere(A_4, point_size, 0.5, 0.5, 0.5, "A_4"); // 灰色
                viewer->addSphere(A_5, point_size, 1.0, 0.5, 0.0, "A_5"); // 橙色
                viewer->addSphere(A_6, point_size, 1.0, 1.0, 1.0, "A_6"); // 白色
                viewer->addLine(A_1, A_2, 0.5, 0.0, 0.5, "A_line12"); // 深紫色
                viewer->addLine(A_2, A_3, 0.5, 0.0, 0.5, "A_line23"); // 深紫色
                viewer->addLine(A_3, A_4, 0.5, 0.0, 0.5, "A_line34"); // 深紫色
                viewer->addLine(A_4, A_5, 0.5, 0.0, 0.5, "A_line45"); // 深紫色
                viewer->addLine(A_5, A_6, 0.5, 0.0, 0.5, "A_line56"); // 深紫色
                viewer->addLine(A_6, A_1, 0.5, 0.0, 0.5, "A_line61"); // 深紫色

                // 绘制C_1到C_6
                viewer->addSphere(C_1, point_size, 1.0, 0.0, 0.0, "C_1"); // 红色    
                viewer->addSphere(C_2, point_size, 1.0, 1.0, 0.0, "C_2"); // 黄色
                viewer->addSphere(C_3, point_size, 0.0, 0.0, 1.0, "C_3"); // 蓝色
                viewer->addSphere(C_4, point_size, 0.5, 0.5, 0.5, "C_4"); // 灰色
                viewer->addSphere(C_5, point_size, 1.0, 0.5, 0.0, "C_5"); // 橙色
                viewer->addSphere(C_6, point_size, 1.0, 1.0, 1.0, "C_6"); // 白色
                viewer->addLine(C_1, C_2, 0.5, 0.5, 0.0, "C_line12"); // 深黄色
                viewer->addLine(C_2, C_3, 0.5, 0.5, 0.0, "C_line23"); // 深黄色
                viewer->addLine(C_3, C_4, 0.5, 0.5, 0.0, "C_line34"); // 深黄色
                viewer->addLine(C_4, C_5, 0.5, 0.5, 0.0, "C_line45"); // 深黄色
                viewer->addLine(C_5, C_6, 0.5, 0.5, 0.0, "C_line56"); // 深黄色
                viewer->addLine(C_6, C_1, 0.5, 0.5, 0.0, "C_line61"); // 深黄色
                // 绘制相机中心
                viewer->addSphere(camera_center, point_size, 1.0, 1.0, 1.0, "camera_center"); // 白色

                // 绘制B_1到B_6
                viewer->addSphere(B_1, point_size-3, 1.0, 0.0, 0.0, "B_1"); // 红色    
                viewer->addSphere(B_2, point_size-3, 1.0, 1.0, 0.0, "B_2"); // 黄色
                viewer->addSphere(B_3, point_size-3, 0.0, 0.0, 1.0, "B_3"); // 蓝色
                viewer->addSphere(B_4, point_size-3, 0.5, 0.5, 0.5, "B_4"); // 灰色
                viewer->addSphere(B_5, point_size-3, 1.0, 0.5, 0.0, "B_5"); // 橙色
                viewer->addSphere(B_6, point_size-3, 1.0, 1.0, 1.0, "B_6"); // 白色
                viewer->addLine(B_1, A_1, 0.5, 0.5, 0.0, "BA_line1"); // 深黄色
                viewer->addLine(B_2, A_2, 0.5, 0.5, 0.0, "BA_line2"); // 深黄色
                viewer->addLine(B_3, A_3, 0.5, 0.5, 0.0, "BA_line3"); // 深黄色
                viewer->addLine(B_4, A_4, 0.5, 0.5, 0.0, "BA_line4"); // 深黄色
                viewer->addLine(B_5, A_5, 0.5, 0.5, 0.0, "BA_line5"); // 深黄色
                viewer->addLine(B_6, A_6, 0.5, 0.5, 0.0, "BA_line6"); // 深黄色

                // 绘制B_i到C_i
                viewer->addLine(B_1, C_1, 0.5, 0.5, 0.0, "BC_line1"); // 深黄色
                viewer->addLine(B_2, C_2, 0.5, 0.5, 0.0, "BC_line2"); // 深黄色
                viewer->addLine(B_3, C_3, 0.5, 0.5, 0.0, "BC_line3"); // 深黄色
                viewer->addLine(B_4, C_4, 0.5, 0.5, 0.0, "BC_line4"); // 深黄色
                viewer->addLine(B_5, C_5, 0.5, 0.5, 0.0, "BC_line5"); // 深黄色
                viewer->addLine(B_6, C_6, 0.5, 0.5, 0.0, "BC_line6"); // 深黄色

                // viewer->addLine(vertices[0], vertices[1], 1.0, 0.0, 0.0, "line01"); // 红色
                // viewer->addLine(vertices[1], vertices[2], 0.0, 1.0, 0.0, "line12"); // 绿色
                // viewer->addLine(vertices[2], vertices[3], 0.0, 0.0, 1.0, "line23"); // 蓝色
                // viewer->addLine(vertices[3], vertices[0], 1.0, 1.0, 0.0, "line30"); // 黄色

                // viewer->addLine(vertices[4], vertices[5], 1.0, 0.0, 1.0, "line45"); // 紫色
                // viewer->addLine(vertices[5], vertices[6], 0.0, 1.0, 1.0, "line56"); // 青色
                // viewer->addLine(vertices[6], vertices[7], 1.0, 1.0, 1.0, "line67"); // 白色
                // viewer->addLine(vertices[7], vertices[4], 0.5, 0.5, 0.5, "line74"); // 灰色

                // viewer->addLine(vertices[0], vertices[4], 1.0, 0.5, 0.0, "line04"); // 橙色
                // viewer->addLine(vertices[1], vertices[5], 0.5, 0.0, 0.5, "line15"); // 深紫色
                // viewer->addLine(vertices[2], vertices[6], 0.0, 0.5, 0.5, "line26"); // 深青色
                // viewer->addLine(vertices[3], vertices[7], 0.5, 0.5, 0.0, "line37"); // 深黄色

                // 添加坐标轴
                viewer->addCoordinateSystem(1.0);

                // Spin once to update the viewer
                viewer->spinOnce(10); // 较小的spin时间
            }
        }
        
    }

private:
    pcl::visualization::PCLVisualizer::Ptr viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("xian_duoji_coordidate_model");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::vector<pcl::PointXYZ> vertices;
    double point_size = 5;
    pcl::PointXYZ X = pcl::PointXYZ(90, 0, 0);
    pcl::PointXYZ Y = pcl::PointXYZ(0, 90, 0);
    pcl::PointXYZ Z = pcl::PointXYZ(0, 0, 90);
    pcl::PointXYZ O = pcl::PointXYZ(0, 0, 0);

    double input_alpha = 0;
    double input_beta = 0;
    double input_gamma = 0;
    double input_x = 0;
    double input_y = 0;
    double input_z = 100;

    // 传感器中心与动平台中心的偏差
    double sensor_offset_x = 0;
    double sensor_offset_y = 0;
    double sensor_offset_z = -10;



    const double PI = 3.1415926;
    double R = 83.5;  // 静平台的半径
    double MR = 65;   //动平台的半径
    double LA = 24;   //舵机摇臂的长度
    double LB = 160;  //连杆长度
    double constraint_angle_delta = 18; //舵臂铰链约束角
    double constraint_angle_zata = 90; //动平台铰链约束角
    //解算的中间量
    double LCs_1 = 0;
    double LCs_2 = 0;
    double LCs_3 = 0;
    double LCs_4 = 0;
    double LCs_5 = 0;
    double LCs_6 = 0;

    pcl::PointXYZ A_is = pcl::PointXYZ(0, R, 0);   //A_is舵机坐标系的A点

    pcl::PointXYZ P_d = pcl::PointXYZ(-LA, 0, 0);  //P_d中间量

    pcl::PointXYZ camera_center = pcl::PointXYZ(0, 0, 0);   //用于可视化的相机中心（相机中心在静平台系的坐标）

    //C_im是C点在动平台坐标系的坐标
    pcl::PointXYZ C_1m = pcl::PointXYZ(MR*sin(20 * PI / 180.0) + sensor_offset_x,  
                                       MR*cos(20 * PI / 180.0) + sensor_offset_y, 
                                       0 + sensor_offset_z);
    pcl::PointXYZ C_2m = pcl::PointXYZ(-MR*sin(20 * PI / 180.0)+ sensor_offset_x, 
                                       MR*cos(20 * PI / 180.0) + sensor_offset_y, 
                                       0 + sensor_offset_z);
    pcl::PointXYZ C_3m = pcl::PointXYZ(-MR*cos(10 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(10 * PI / 180.0) + sensor_offset_y, 
                                       0 + sensor_offset_z);
    pcl::PointXYZ C_4m = pcl::PointXYZ(-MR*cos(50 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(50 * PI / 180.0) + sensor_offset_y, 
                                       0 + sensor_offset_z);
    pcl::PointXYZ C_5m = pcl::PointXYZ(MR*cos(50 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(50 * PI / 180.0) + sensor_offset_y, 
                                       0 + sensor_offset_z);
    pcl::PointXYZ C_6m = pcl::PointXYZ(MR*cos(10 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(10 * PI / 180.0) + sensor_offset_y, 
                                       0 + sensor_offset_z);
    
    pcl::PointXYZ camera_center_m = pcl::PointXYZ(0, 0, 0);     //相机中心在动平台坐标系的坐标

    //theta_i是舵机坐标系转化成静平台坐标系绕Z轴的旋转角
    double theta_1 = -21.06;
    double theta_2 = 21.06;
    double theta_3 = 98.94;
    double theta_4 = 141.06;
    double theta_5 = -141.06;
    double theta_6 = -98.94;
    //phi_i是舵臂坐标系转化成舵机坐标系绕Z轴的旋转角
    double phi_1 = 21.06;
    double phi_2 = 158.94;
    double phi_3 = 21.06;
    double phi_4 = 158.94;
    double phi_5 = 21.06;
    double phi_6 = 158.94;
    //omega_i是舵机的旋转角度
    double omega_1 = 0;
    double omega_2 = 0;
    double omega_3 = 0;
    double omega_4 = 0;
    double omega_5 = 0;
    double omega_6 = 0;


    //**************************** 解算中间量 ****************************
    double a_1 = 0;
    double a_2 = 0;
    double a_3 = 0;
    double a_4 = 0;
    double a_5 = 0;
    double a_6 = 0;

    double b_1 = 0;
    double b_2 = 0;
    double b_3 = 0;
    double b_4 = 0;
    double b_5 = 0;
    double b_6 = 0;

    double c_1 = 0;
    double c_2 = 0;
    double c_3 = 0;
    double c_4 = 0;
    double c_5 = 0;
    double c_6 = 0;

    double M1_11 = 0;
    double M2_11 = 0;
    double M3_11 = 0;
    double M4_11 = 0;
    double M5_11 = 0;
    double M6_11 = 0;

    double M1_21 = 0;
    double M2_21 = 0;
    double M3_21 = 0;
    double M4_21 = 0;
    double M5_21 = 0;
    double M6_21 = 0;

    double ac_11 = 0;
    double ac_21 = 0;
    double ac_31 = 0;
    double ac_41 = 0;
    double ac_51 = 0;
    double ac_61 = 0;

    double ac_12 = 0;
    double ac_22 = 0;
    double ac_32 = 0;
    double ac_42 = 0;
    double ac_52 = 0;
    double ac_62 = 0;

    double ac_13 = 0;
    double ac_23 = 0;
    double ac_33 = 0;
    double ac_43 = 0;
    double ac_53 = 0;
    double ac_63 = 0;

    pcl::PointXYZ C_1A_1 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_2A_2 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_3A_3 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_4A_4 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_5A_5 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_6A_6 = pcl::PointXYZ(0, 0, 0);
    //**************************** 解算中间量 ****************************
    
    //A_i、B_i、C_i在静平台坐标系的坐标，B_id是B点在舵臂坐标系的坐标，B_is是B点在舵机坐标系的坐标
    pcl::PointXYZ A_1 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ A_2 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ A_3 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ A_4 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ A_5 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ A_6 = pcl::PointXYZ(0, 0, 0);

    pcl::PointXYZ C_1 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_2 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_3 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_4 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_5 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_6 = pcl::PointXYZ(0, 0, 0);

    pcl::PointXYZ B_1d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_2d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_3d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_4d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_5d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_6d = pcl::PointXYZ(0, 0, 0);

    pcl::PointXYZ B_1s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_2s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_3s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_4s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_5s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_6s = pcl::PointXYZ(0, 0, 0);

    pcl::PointXYZ B_1 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_2 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_3 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_4 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_5 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_6 = pcl::PointXYZ(0, 0, 0);


    // 舵机上的铰链约束角计算
    // 声明delta_i
    double delta_1 = 0;
    double delta_2 = 0;
    double delta_3 = 0;
    double delta_4 = 0;
    double delta_5 = 0;
    double delta_6 = 0;
    // D_i是舵臂连杆上的D点在静平台坐标系的坐标
    pcl::PointXYZ D_1 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_2 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_3 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_4 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_5 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_6 = pcl::PointXYZ(0, 0, 0);
    // D_is是舵臂连杆上的D点在舵机坐标系的坐标
    pcl::PointXYZ D_1s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_2s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_3s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_4s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_5s = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_6s = pcl::PointXYZ(0, 0, 0);
    // D_id是舵臂连杆上的D点在舵臂坐标系的坐标
    pcl::PointXYZ D_1d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_2d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_3d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_4d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_5d = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ D_6d = pcl::PointXYZ(0, 0, 0);
    //
    double yueshu_delta_offset135 = -1;
    double yueshu_delta_offset246 = 1;
    pcl::PointXYZ TD_id135 = pcl::PointXYZ(-LA, yueshu_delta_offset135, 0);
    pcl::PointXYZ TD_id246 = pcl::PointXYZ(-LA, yueshu_delta_offset246, 0);

    // 动平台上的铰链约束角计算: zeta_1
    // 声明zeta_i
    double zeta_1 = 0;
    double zeta_2 = 0;
    double zeta_3 = 0;
    double zeta_4 = 0;
    double zeta_5 = 0;
    double zeta_6 = 0;

    // E_im是铰链座轴上的E点在动平台坐标系的坐标
    double yueshu_zeta_offset = -1;
    pcl::PointXYZ E_1m = pcl::PointXYZ(MR*sin(20 * PI / 180.0) + sensor_offset_x,  
                                       MR*cos(20 * PI / 180.0) + sensor_offset_y, 
                                       yueshu_zeta_offset + sensor_offset_z);
    pcl::PointXYZ E_2m = pcl::PointXYZ(-MR*sin(20 * PI / 180.0)+ sensor_offset_x, 
                                       MR*cos(20 * PI / 180.0) + sensor_offset_y, 
                                       yueshu_zeta_offset + sensor_offset_z);
    pcl::PointXYZ E_3m = pcl::PointXYZ(-MR*cos(10 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(10 * PI / 180.0) + sensor_offset_y, 
                                       yueshu_zeta_offset + sensor_offset_z);
    pcl::PointXYZ E_4m = pcl::PointXYZ(-MR*cos(50 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(50 * PI / 180.0) + sensor_offset_y, 
                                       yueshu_zeta_offset + sensor_offset_z);
    pcl::PointXYZ E_5m = pcl::PointXYZ(MR*cos(50 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(50 * PI / 180.0) + sensor_offset_y, 
                                       yueshu_zeta_offset + sensor_offset_z);
    pcl::PointXYZ E_6m = pcl::PointXYZ(MR*cos(10 * PI / 180.0)+ sensor_offset_x, 
                                       -MR*sin(10 * PI / 180.0) + sensor_offset_y, 
                                       yueshu_zeta_offset + sensor_offset_z);
    // E_i是铰链座轴上的E点在静平台坐标系的坐标
    pcl::PointXYZ E_1 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ E_2 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ E_3 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ E_4 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ E_5 = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ E_6 = pcl::PointXYZ(0, 0, 0);

    // 向量CE与CB声明
    pcl::PointXYZ C_1E_1_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_2E_2_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_3E_3_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_4E_4_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_5E_5_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_6E_6_vec = pcl::PointXYZ(0, 0, 0);

    pcl::PointXYZ C_1B_1_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_2B_2_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_3B_3_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_4B_4_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_5B_5_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ C_6B_6_vec = pcl::PointXYZ(0, 0, 0);

    // 向量BD与BC声明
    pcl::PointXYZ B_1D_1_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_2D_2_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_3D_3_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_4D_4_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_5D_5_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_6D_6_vec = pcl::PointXYZ(0, 0, 0);

    pcl::PointXYZ B_1C_1_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_2C_2_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_3C_3_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_4C_4_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_5C_5_vec = pcl::PointXYZ(0, 0, 0);
    pcl::PointXYZ B_6C_6_vec = pcl::PointXYZ(0, 0, 0);
    
};

int main(int argc, char** argv)
{
    // Initialize and name node
    ros::init(argc, argv, "xian_duoji_coordidate_model");
    XianDuojiCoordidateModel xian_duoji_coordidate_model;

    // 创建一个ROS节点句柄
    ros::NodeHandle nh_2;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    return 0;
}