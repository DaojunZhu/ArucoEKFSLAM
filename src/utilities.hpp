#include <Eigen/Dense>

//calculate the subtraction between two vectors
Eigen::Vector3d vector_sub(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d c;
    c(0) = a(0) - b(0);
    c(1) = a(1) - b(1);
    c(2) = a(2) - b(2);
    return c;
}

//normalize angle,normalize z between -pi and pi
double normalize(double z)
{
    return atan2(sin(z),cos(z));
}

double normalize2(double z)
{
    double z_norm;
    while(z>M_PI){
        z = z - 2*M_PI;
    }
    while(z<-M_PI){
        z = z + 2*M_PI;
    }
    z_norm = z;
    return z_norm;
}

//calculate the difference between two angles
double angle_diff(double a ,double b)
{
    double d1,d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1>0)
        d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return(d1);
    else
        return(d2);
}

//TODO: try to understand code block here. 
//There is a difference as Prop Rob in sample algorithm,which use variance sigma^2 as parameter
//Draw randomly from a zero-mean Gaussian distribution, with standard deviation sigma
double sample_gaussian(double sigma)
{
    double x1,x2,w,r;
    do{
        do{ r = drand48();}while(r==0.0);
        x1 = 2.0 * r -1.0;
        do{ r = drand48();}while(r==0.0);
        x2 = 2.0*r -1.0;
        w = x1*x1 + x2*x2;
    }while(w>1.0||w==0.0);
    return(sigma*x2*sqrt(-2.0*log(w)/w));
}

