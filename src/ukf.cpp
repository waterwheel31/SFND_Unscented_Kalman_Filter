#include <iostream> 
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

   // initial state vector
  n_x_ = 5;
  x_ = VectorXd(n_x_); 
  x_.fill(0.0);
  n_aug_ = n_x_ + 2;   //  adding 2 noise factors
  n_sig_ = n_aug_ * 2 + 1; 
  lambda_ = 3 - n_aug_;

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
 
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.02; // original 30
  
  /**
   * DO NOT MODIFY measurement noise values below. These are provided by the sensor manufacturer.
  */
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // weights for sigma points initialization 
  weights_ = VectorXd( n_sig_);
  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  X_sig_pred_ = MatrixXd(n_x_, n_sig_);
  
  // lidar measurament noise conv matrix 
  R_lidar_ = MatrixXd(2,2);
  R_lidar_ <<  std_laspx_ * std_laspx_,   0, 
               0,                       std_laspy_ * std_laspy_; 

 // Radar measurament noise conv matrix 
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_ , 0                        , 0,
              0                     ,std_radphi_ * std_radphi_ , 0,
              0                     , 0                        , std_radrd_ * std_radrd_ ; 


  is_initialized_ = false;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // Initialization 
  if (!is_initialized_){

      //std::cout << "sensor type: " << meas_package.sensor_type_  << std::endl; 

      if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){

          double x = meas_package.raw_measurements_[0];
          double y = meas_package.raw_measurements_[1];

          x_ << x, y, 0, 0, 0; 

          time_us_ = meas_package.timestamp_;
          is_initialized_ = true; 

          std::cout << "initial process measurement(LASER) " << std::endl;
          std::cout <<  " x_ : " << std::endl << x_  << std::endl;
        }

      if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
        
          // raw measurament 
          double range = meas_package.raw_measurements_[0];
          double phi = meas_package.raw_measurements_[1];   // angle 
          double r_velocity = meas_package.raw_measurements_[2]; 
          
          // converting 
          double x = range * cos(phi); 
          double y = range * sin(phi);
          //double v = r_velocity; 

          double vx = r_velocity * cos(phi);
          double vy = r_velocity * sin(phi);
          double v =   sqrt(vx * vx + vy * vy);//r_velocity;

          x_ << x, y, v, 0, 0; 

          time_us_ = meas_package.timestamp_;
          is_initialized_ = true; 

          //std::cout << "initial process measurement  range : " << range << "  phi:" << phi << std::endl;
          std::cout << "initial process measurement (RADAR) x_ : " << std::endl << x_  << std::endl;
      }

  }
  // std::cout << "process measurement  x_ : " << std::endl << x_  << std::endl;

  // Prediction

  std::cout << "meas_package.timestamp_: "  << meas_package.timestamp_ << "  time_us_: " << time_us_ << std::endl; 

  // double delta_t = (meas_package.timestamp_ - time_us_) * 1e-6;
  const double delta_t{ (meas_package.timestamp_ - time_us_) / 1000000.0 };

  if (delta_t > 0){

      time_us_ = meas_package.timestamp_; 

      /*
      while (delta_t > 0.1) {
            constexpr double delta_t_temp = 0.05;
            Prediction(delta_t_temp);
            delta_t -= delta_t_temp;
        }
      */


      Prediction(delta_t); 

      //std::cout << meas_package.sensor_type_;
      // Update measurament 
      if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
        UpdateLidar(meas_package);
        //std::cout << " Update Lidar" << std::endl;
      }
      if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
        UpdateRadar(meas_package);
        //std::cout << " Update Radar" << std::endl;
      }
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
    //std::cout << "Prediction()" << std::endl;
     // Augumentation and generation of sigma points 

    std::cout << "delta_t: " << delta_t << std::endl;  

    X_sig_aug_ = MatrixXd(n_aug_, n_sig_);     //Sigma points.  7 x 15 
    X_sig_aug_.fill(0.0); 

    // std::cout << "Prediction x_ : " <<  std::endl << x_ << std::endl; 
    x_aug_ = VectorXd(n_aug_);      // state vector 
    x_aug_.fill(0.0);
    x_aug_.head(n_x_) = x_; 

    std::cout << "prediction " << " x: " << x_[0] << " " << x_[1]  << " " << x_[2]  << " " << x_[3]  << " " << x_[4] << std::endl;


    P_aug_ = MatrixXd(n_aug_, n_aug_);   // Covariance matrix 
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(n_x_, n_x_) = P_; 

    //std::cout << "P_ :"  << std::endl << P_ << std::endl;  


    P_aug_(n_x_, n_x_) = std_a_ * std_a_;
    P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_; 

    MatrixXd L = P_aug_.llt().matrixL();

    X_sig_aug_.col(0) = x_aug_;    // 1st column 
    for (int i = 0; i < n_aug_; i++){
      X_sig_aug_.col(i+1)          = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);    // 1st set 
      X_sig_aug_.col(i+1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);  // 2nd set 

      std::cout << "L.col(i): " << L.col(i)[0] << " " << L.col(i)[1] << " "<< L.col(i)[2] << " " << L.col(i)[3] << " " <<  L.col(i)[4] << std::endl;
    }

    std::cout << "x_aug_: " << x_aug_[0] << " "  << x_aug_[1] << " "  << x_aug_[2] << " "  << x_aug_[3] << " "  << x_aug_[4] << " "  << x_aug_[5] << " "  << x_aug_[6] << std::endl; 

    // Predicting sigma points   
    //std::cout << "Prediction() -predicting sigma points" << std::endl;
    X_sig_pred_ = MatrixXd(n_x_, n_sig_);
    
    std::cout << "X_sig_aug: " << std::endl << X_sig_aug_ << std::endl;  

    for (int i = 0; i < n_sig_; i++){

      double p_x = X_sig_aug_(0,i); 
      double p_y = X_sig_aug_(1,i); 
      double v = X_sig_aug_(2,i); 
      double yaw = X_sig_aug_(3,i); 
      double yawd = X_sig_aug_(4,i); 
      double nu_a = X_sig_aug_(5,i);   // noise 
      double nu_yawdd = X_sig_aug_(6,i);  // noise 

      // std::cout << "nu_yawdd: " << nu_yawdd << std::endl; 

      double px_p, py_p;

      if (std::fabs(yawd) > 0.001){
        px_p = p_x + v / yawd + (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd + (cos(yaw) - cos(yaw + yawd * delta_t));
      } else { 
        px_p = p_x + v * delta_t * cos(yaw); 
        py_p = p_y + v * delta_t * sin(yaw);
      }

      std::cout << "px_p:" << px_p << " v :" << v << std::endl; 

      double v_p = v;

      double yaw_p = yaw + yawd * delta_t;
      double yawd_p = yawd; 

      // adding noise 
      px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
      py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
      v_p = v_p + nu_a * delta_t;
      yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
      yawd_p = yawd_p + nu_yawdd * delta_t;

      //std::cout << "delta: " << delta_t << std::endl; 

      // adding to the matrix 
      X_sig_pred_(0,i) = px_p; 
      X_sig_pred_(1,i) = py_p; 
      X_sig_pred_(2,i) = v_p; 
      X_sig_pred_(3,i) = yaw_p; 
      X_sig_pred_(4,i) = yawd_p; 

      //std::cout << "delta_t: " << delta_t << std::endl;  

      std::cout << "nu_a: " << nu_a << "| px_p: "<< px_p << " |py_p:  " << py_p << " |v_p:  " << v_p << " |yaw_p: " << yaw_p  << " |yawd_p: " << yawd_p << std::endl;  //  "  delta_t: " << delta_t <<  std::endl; 
      //std::cout << "X_sig_pred col: " << i << std::endl << X_sig_pred_.col(i) << std::endl; 

      // predicting state mean
      //std::cout << "Prediction() -predicting state mean" << std::endl;
      x_.fill(0.0); 
      for (int i = 0; i < n_sig_; i++){
        x_ = x_ + weights_(i) * X_sig_pred_.col(i); 
        //std::cout << "prediction " << " x: " << x_[0] << " " << x_[1]  << " " << x_[2]  << " " << x_[3]  << " " << x_[4] <<  "    X_sig_pred_.col(i) " <<  X_sig_pred_.col(i)[0] << " " <<  X_sig_pred_.col(i)[1]  << " " <<  X_sig_pred_.col(i)[2] << " " << X_sig_pred_.col(i)[3] << " " <<  X_sig_pred_.col(i)[4]     << std::endl;  
        
        counter ++;
        //std::cout << "counter:" << counter << std::endl;
        // if (counter >= 10000) {exit(1); }
      }

      // predicting state covariance 
      P_.fill(0.0);
      for (int i = 0; i < n_sig_; i++){
        VectorXd x_diff = X_sig_pred_.col(i) - x_; 

        //std::cout << "x_sig_pred_col(i): " << X_sig_pred_.col(i)[0] << " " << X_sig_pred_.col(i)[1] << " " << X_sig_pred_.col(i)[2] << " " << X_sig_pred_.col(i)[3] << " " << X_sig_pred_.col(i)[4] << std::endl; 
        //std::cout << "x_: " << x_[0] << " " << x_[1] << " " << x_[2] << " " << x_[3] << " " << x_[4] << std::endl; 

        //while(x_diff(3) > M_PI) {x_diff(3) -= 2.*M_PI; }
        //while(x_diff(3) < -M_PI) {x_diff(3) += 2.*M_PI; }

        P_ = P_ + weights_(i) * (x_diff * x_diff.transpose());

        //std::cout << "x_diff: " << x_diff[0] << " " << x_diff[1] << " " << x_diff[2] << " " << x_diff[3] << " " << x_diff[4] << std::endl; 

      }

      //std::cout << "P_  after predicting state covariance:"  << std::endl << P_ << std::endl;  

    }

   //std::cout << "x_ (prediction end prediction) : " <<  std::endl << x_ << std::endl; 
   /*
  */


}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  VectorXd z_ = meas_package.raw_measurements_;


  int n_z_ = 2;
  MatrixXd Z_sig = MatrixXd(n_z_, n_sig_);

  for(int i = 0; i < n_sig_; i++)
  {
    Z_sig(0, i) = X_sig_pred_(0, i);
    Z_sig(1, i) = X_sig_pred_(1, i);
  }

  //std::cout << " mean " << std::endl; 
  
  VectorXd z_pred_= VectorXd(n_z_);
  z_pred_.fill(0.0);
  for(int i = 0; i < n_sig_; i++)
  {
    z_pred_ = z_pred_ + weights_(i) * Z_sig.col(i);
  }
  
  //std::cout << " covariance" << std::endl; 
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);
  for(int i = 0; i < n_sig_; i++)
  {
    VectorXd z_diff = Z_sig.col(i) - z_pred_;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //std::cout << " adding noise"  << std::endl; 
  S = S + R_lidar_;

  //update
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);

  for(int i = 0; i < n_sig_; i++)
  {
    VectorXd x_diff = X_sig_pred_.col(i) - x_;
    VectorXd z_diff = Z_sig.col(i) - z_pred_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z_ - z_pred_;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */


  //std::cout << "z_ " << std::endl << z_ << std::endl;

  //std::cout << "X_sig_pred_ " << std::endl << X_sig_pred_ << std::endl;

  // transforming sigma points into measurement space 
   //std::cout << "transforming sigma points into measurement space  " << std::endl;
   int n_z_ = 3;  
   MatrixXd Z_sig = MatrixXd(n_z_, n_sig_); 

   for (int i = 0; i < n_sig_; i++){ 
      double p_x = X_sig_pred_(0, i);
      double p_y = X_sig_pred_(1, i);
      double v = X_sig_pred_(2, i);
      double yaw = X_sig_pred_(3, i);
      
      double v_x = cos(yaw) * v;
      double v_y = sin(yaw) * v;

      // measurement model

      double rho = sqrt(p_x*p_x + p_y*p_y); 
      double phi = atan2(p_y,p_x);  
      double rho_dot = v; //0.0; 

  

      Z_sig(0,i) = rho;                  // rho
      Z_sig(1,i) = phi;                         // phi
      if (std::fabs(rho) > 0.001){
        Z_sig(2,i) = (p_x*v_x + p_y*v_y) / rho;   // rho_dot
        //std::cout <<  "Z_sig(2,i) count, value = "  << Z_sig(2,i)  << "p_x: "<< p_x << " p_y: " << p_y <<  " v:" << v   << " v_x: "  << v_x << " v_y: " << v_y  << " yaw: "<< yaw << std::endl;
      } else {
         Z_sig(2,i) = (p_x*v_x + p_y*v_y) / 0.001; 
        //std::cout <<  "Z_sig(2,i) not count"  << std::endl;
      }


   }

  //std::cout << "Z_sig " << std::endl << Z_sig << std::endl;

   //std::cout << "calculating the means  " << std::endl;
   // calculating the means 
   VectorXd z_pred_ = VectorXd(n_z_); 
   z_pred_.fill(0.0);


   // calculating the mean vector
   for (int i = 0; i < n_sig_; i++){
     //std::cout << "i:" << i << std::endl; 
     //std::cout << "weights(i):" << weights_(i) << std::endl; 
     //std::cout << "Z_sig.col(i):" << Z_sig.col(i) << std::endl; 

     z_pred_ = z_pred_ + weights_(i) * Z_sig.col(i);
   }

   //std::cout << "z_pred " << std::endl << z_pred << std::endl;

   // calculating the covariance matrix S
   //std::cout << "calculating the covariance matrix S  " << std::endl;
   MatrixXd S = MatrixXd(n_z_, n_z_); 
   S.fill(0.0);

   for (int i = 0; i < n_sig_; i++){
      VectorXd z_diff = Z_sig.col(i) - z_pred_;

      // angle normalization
      //while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
      //while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

      S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  //std::cout << "add measurement noise covariance matrix  " << std::endl;
  S = S + R_radar_;


  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_); 
  //std::cout << "create matrix for cross correlation Tc  " << std::endl;

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Z_sig.col(i) - z_pred_;
    // angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = X_sig_pred_.col(i) - x_;
    // angle normalization
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }


  // Kalman gain K;
 // std::cout << "Kalman gain K  " << std::endl;
  MatrixXd K = Tc * S.inverse();
  //std::cout << "K: " << std::endl << K << std::endl;


  // update state mean and covariance matrix
  //std::cout << "update state mean and covariance matrix  " << std::endl;
  VectorXd z_ = meas_package.raw_measurements_;

  std::cout << " z_ : " <<  z_[0] << " " << z_[1] << " " << z_[2]  << std::endl; 


  VectorXd z_diff = z_ - z_pred_;
  std::cout << " z_diff : " <<  z_diff[0] << " " << z_diff[1] << " " << z_diff[2]  << std::endl; 

  //std::cout << "z_diff: " << std::endl << z_diff  << std::endl; 

  // angle normalization
  //std::cout << "angle normalization  " << std::endl;
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();


  std::cout << "x after " << " x: " << x_[0] << " " << x_[1]  << " " << x_[2]  << " " << x_[3]  << " " << x_[4] << std::endl;

  //std::cout << "X differential :" << K * z_diff << std::endl;  
  //std::cout << "Update end  x_ " << std::endl << x_ << std::endl; 


}