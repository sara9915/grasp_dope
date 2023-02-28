#include <math.h> /* pow */
#include <eigen3/Eigen/Geometry>
#include <vector>

void update_coeff(Eigen::Matrix<double, 6, 7> &coeff, const double &tf, const std::vector<double> &qi, const std::vector<double> &qi_dot, const std::vector<double> &qi_dot_dot, const std::vector<double> &qf, const std::vector<double> &qf_dot, const std::vector<double> &qf_dot_dot, const double &num_points_traj)
{
    double tf_2 = pow(tf, 2);
    double tf_3 = pow(tf, 3);
    double tf_4 = pow(tf, 4);
    double tf_5 = pow(tf, 5);

    Eigen::Matrix3d A;
    A << 20.0 * tf_3, 12.0 * tf_2, 6.0 * tf,
        5.0 * tf_4, 4.0 * tf_3, 3.0 * tf_2,
        tf_5, tf_4, tf_3;

    Eigen::Vector3d b;
    Eigen::Vector3d coeff_calc;

    for (int j = 0; j < coeff.cols(); j++)
    {
        coeff(5, j) = qi[j];               // a0
        coeff(4, j) = qi_dot[j];           // a1
        coeff(3, j) = qi_dot_dot[j] / 2.0; // a2

        b(0) = qf_dot_dot[j] - qi_dot_dot[j];
        b(1) = qf_dot[j] - qi_dot[j] - qi_dot_dot[j] * tf;
        b(2) = qf[j] - qi[j] - qi_dot[j] * tf - qi_dot_dot[j] * tf_2 / 2.0;

        coeff_calc = A.inverse() * b;

        coeff(0, j) = coeff_calc[0]; // a5
        coeff(1, j) = coeff_calc[1]; // a4
        coeff(2, j) = coeff_calc[2]; // a3
    }
}

double quintic_q(const double &t, const Eigen::Matrix<double, 6, 7> &coeff, const int &i)
{
    double t_2 = pow(t, 2);
    double t_3 = pow(t, 3);
    double t_4 = pow(t, 4);
    double t_5 = pow(t, 5);

    return (coeff(0, i) * t_5 + coeff(1, i) * t_4 + coeff(2, i) * t_3 + coeff(3, i) * t_2 + coeff(4, i) * t + coeff(5, i));
}

double quintic_qdot(const double &t, const Eigen::Matrix<double, 6, 7> &coeff, const int &i)
{
    double t_2 = pow(t, 2);
    double t_3 = pow(t, 3);
    double t_4 = pow(t, 4);
    double t_5 = pow(t, 5);

    return (coeff(0, i) * 5 * t_4 + coeff(1, i) * 4 * t_3 + coeff(2, i) * 3 * t_2 + coeff(3, i) * 2 * t + coeff(4, i));
}
