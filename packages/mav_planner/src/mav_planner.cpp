#include "mav_planner.h"

const double fs = 100;
const double Ts = 0.01;


double rep_gain_p = 0;
double gain_p = 0;
double gain_d = 0.1;

//--Utilizzato per grafici
std_msgs::Float32MultiArray force_potential_msg;
//--


MAVPLN::MAVPLN() {  

    _min_obs_dist=100;
    _att_pot = 0;
    _rep_pot = 0;
    

    _first_laser = false;
    _first_odom = false;
    _found_obstacle = false;
    _stop_laser_update = false;
    _stop_odom_update = false;
    _landing = false;
    _emp_obs_distant=false;
    _emp_obs_near= false; 
    _dest_reached = false;
    _emp_obs_approaching = false;
    _emp_obs_crossing = false;
    _add_distant = false;
    _add_near = false;
    
    _takeoff_ok = false;


    _TF_zero = false;
    _dest_free = false;
    _tot_pot_not_zero = false;

    _des_p = Eigen::MatrixXd::Zero(3,1);            
    _des_dp = Eigen::MatrixXd::Zero(3,1);           
    _des_ddp = Eigen::MatrixXd::Zero(3,1);          
    _des_yaw = 0;                                   
    _des_dyaw = 0;                                  
    _des_ddyaw = 0;                                 

    _emp_pos_wned = Eigen::MatrixXd::Zero(3,1);     
    _emp_ori_wned = Eigen::MatrixXd::Zero(3,1);       
    _emp_pos_ll = Eigen::MatrixXd::Zero(3,1);               
    _emp_ori_bl = Eigen::MatrixXd::Zero(3,1);             

    _curr_p = Eigen::MatrixXd::Zero(3,1);;   
    _curr_eta = Eigen::MatrixXd::Zero(3,1);
    _curr_dp = Eigen::MatrixXd::Zero(3,1);         
    _curr_deta = Eigen::MatrixXd::Zero(3,1); 
    _Q = Eigen::MatrixXd::Zero(3,3);
    _wbb = Eigen::MatrixXd::Zero(3,1);
    _Rb = Eigen::MatrixXd::Zero(3,3);

    if(_nh.hasParam("arm_length")){_nh.getParam("arm_length",_al);ROS_INFO_STREAM("arm_length\t"<<_al);}

    if(_nh.hasParam("Kp_p")){_nh.getParam("Kp_p",_Kp_p);ROS_INFO_STREAM("_Kp_p\t"<<_Kp_p);}
    if(_nh.hasParam("Kp_o")){_nh.getParam("Kp_o",_Kp_o);ROS_INFO_STREAM("_Kp_o\t"<<_Kp_o);}
    if(_nh.hasParam("Kd_p")){_nh.getParam("Kd_p",_Kd_p);ROS_INFO_STREAM("_Kd_p\t"<<_Kd_p);}
    if(_nh.hasParam("Kd_o")){_nh.getParam("Kd_o",_Kd_o);ROS_INFO_STREAM("_Kd_o\t"<<_Kd_o);}
    if(_nh.hasParam("K_obs")){_nh.getParam("K_obs",_K_obs);ROS_INFO_STREAM("_K_obs\t"<<_K_obs);}
    

    _odom_sub  = _nh.subscribe("/hummingbird/odometry", 1, &MAVPLN::odometry_cb, this);
    _lidar_sub = _nh.subscribe("/laser/scan", 1, &MAVPLN::laser_cb, this);
    _marker_sub = _nh.subscribe("/aruco_single/pose", 1, &MAVPLN::marker_cb, this);

    _plan_pub  = _nh.advertise<custom_msg_pkg::planner_msg>("/planner_des", 1);
    _for_pot_pub = _nh.advertise<std_msgs::Float32MultiArray>("/force_pot", 1);
   
   
    _obs_list.clear();
    _wp_list.clear();
    
    //-- Riempio vettore dei way points (gestione FIFO)
    //worldNED frame
    Eigen::Vector3d p;
    p(0) =  6.0;
    p(1) =  -16.0;
    p(2) = -1.5;
    _wp_list.push_back( p );
    p(0) =  4.0;
    p(1) =  -13.0;
    p(2) = -1.0;
    _wp_list.push_back( p );
    p(0) =  0.0;
    p(1) =  -9.5;
    p(2) = -1.0;
    _wp_list.push_back( p );
    p(0) =  0.0;
    p(1) =  -4.0;
    p(2) = -1.0;
    _wp_list.push_back( p );
    _wp_index = _wp_list.size();
    
    
}


void MAVPLN::odometry_cb( const nav_msgs::Odometry& odom ) {
    //frame_id: "worldNED"
    ////cout<<"odom"<<endl;
    if(!_stop_odom_update){

        _curr_p(0) = odom.pose.pose.position.x;
        _curr_p(1) = odom.pose.pose.position.y;
        _curr_p(2) = odom.pose.pose.position.z;

        _curr_dp(0) = odom.twist.twist.linear.x;
        _curr_dp(1) = odom.twist.twist.linear.y;
        _curr_dp(2) = odom.twist.twist.linear.z;
    
        _wbb(0) = odom.twist.twist.angular.x; // _curr_deta(0) = odom.twist.twist.angular.x;
        _wbb(1) = odom.twist.twist.angular.y; // _curr_deta(1) = odom.twist.twist.angular.y;
        _wbb(2) = odom.twist.twist.angular.z; // _curr_deta(2) = odom.twist.twist.angular.z;

        tf::Quaternion q( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y , odom.pose.pose.orientation.z ,  odom.pose.pose.orientation.w );
        tf::Matrix3x3 Rb_tf(q);
        Rb_tf.getRPY(_curr_eta(0),_curr_eta(1),_curr_eta(2));
        tf::matrixTFToEigen(Rb_tf, _Rb);

        _Q.row(0) << 1,                  0,                  -sin(_curr_eta(1));
        _Q.row(1) << 0,  cos(_curr_eta(0)), cos(_curr_eta(1))*sin(_curr_eta(0));
        _Q.row(2) << 0, -sin(_curr_eta(0)), cos(_curr_eta(1))*cos(_curr_eta(0));

        _curr_deta = _Q.inverse()*_wbb; // _wbb = _Q*_curr_deta;

        _first_odom = true;

    }
    
    ////cout<<"odometry_cb"<<endl;
    ////cout << "_Rb" <<endl<< _Rb<< endl;
    ////cout << "_curr_p" <<endl<< _curr_p<< endl;
    ////cout << "_curr_dp" <<endl<< _curr_dp<< endl;
    ////cout << "_curr_eta" <<endl<< _curr_eta<< endl;
    ////cout << "_curr_deta" <<endl<< _curr_deta<< endl; 
    ////cout<<"_Q"<<endl<<_Q<<endl;
    ////cout<<"_dQ"<<endl<<_dQ<<endl;
    ////cout<<"_wbb"<<endl<<_wbb<<endl;
    

}

void MAVPLN::laser_cb( sensor_msgs::LaserScan laser ){// prof sensor_msgs::LaserScan laser

    //cout<<"LASER"<<endl;

    if( !_stop_laser_update){

        _found_obstacle=false;
        _obs_list.clear();

        if(!(_emp_obs_near || _emp_obs_approaching || _emp_obs_crossing)){

            const double obstacle_threshold = 4;
            int obs_start_ind=0;
            int obs_end_ind=0;
            int index = 0;
            const int end_ind = int(floor(float(PI)/ double(laser.angle_increment)) );

            while (index <= end_ind  ) {

                ////cout<<"index\t"<<index<<endl;

                if(  laser.ranges[index] < obstacle_threshold ) {//ricerco ostacolo
                    _found_obstacle=true;
                    //-- determino estensione ostacolo
                    obs_start_ind = index;
                    while( index < end_ind && laser.ranges[index] < obstacle_threshold ){index++;}
                    obs_end_ind = index;
                    index++;
                    //--
                    //-- Gestisco la presenza di un empty_obstacle, trascurando gli indici che lo interessano
                        if(_emp_obs_distant){

                            double emp_d = sqrt(pow(_emp_pos_ll(0),2) + pow(_emp_pos_ll(1),2));
                            double emp_a = atan(_emp_pos_ll(0)/_emp_pos_ll(1)) - 0.5 * PI; 
                            //ROS_INFO("emp_d %f emp_a %f", emp_d, emp_a);
                            double temp = double(1.5)/(2*emp_d);    //la finestra è larga 1.4 metri, faccio 1.5 
                            if(temp<-1)temp=-1;
                            if(temp>1)temp=1;
                            double emp_angle_width = 2 * asin(temp);
                            int emp_obs_start_ind = int(floor(float(emp_a - 0.5 * emp_angle_width)/ double(laser.angle_increment)) );
                            int emp_obs_end_ind =int(floor(float(emp_a + 0.5 * emp_angle_width)/ double(laser.angle_increment)) );
                            //ROS_INFO("\n EO_s %d EO_e %d", emp_obs_start_ind, emp_obs_end_ind);
                            //ROS_INFO("\n SO_s %d SO_e %d", obs_start_ind, obs_end_ind);

                            if(obs_start_ind < emp_obs_start_ind && emp_obs_start_ind < obs_end_ind && obs_end_ind < emp_obs_end_ind){
                                //ROS_INFO("LASER_1");
                                SolidObstacle(obs_start_ind, emp_obs_start_ind, laser);
                            }
                            else if(emp_obs_start_ind < obs_start_ind && obs_start_ind < emp_obs_end_ind && emp_obs_end_ind < obs_end_ind){
                                //ROS_INFO("LASER_2");
                                SolidObstacle(emp_obs_end_ind, obs_end_ind, laser);
                            }
                            else if( obs_start_ind < emp_obs_start_ind && emp_obs_end_ind < obs_end_ind ){
                                //divido l'ostacolo in due
                                //ROS_INFO("LASER_3");
                                SolidObstacle(obs_start_ind, emp_obs_start_ind, laser);
                                SolidObstacle(emp_obs_end_ind, obs_end_ind, laser);
                            }
                            else if( emp_obs_end_ind < obs_start_ind || emp_obs_start_ind > obs_end_ind){
                                //ROS_INFO("LASER_4");
                                SolidObstacle(obs_start_ind, obs_end_ind, laser);
                            } 
                            ////else(emp_obs_start_ind < obs_start_ind && obs_end_ind < emp_obs_end_ind ){/*scarta ostacolo*/}
                            //ROS_INFO("LASER_5");
                        }else{ 
                            //ROS_INFO("LASER_6");
                            SolidObstacle(obs_start_ind, obs_end_ind, laser);
                        } 
                    //--
                }
                else{
                    index++;
                }
            }

            ////cout<<"END laser"<<endl;
            _first_laser = true;
        
        }

    }

}

void MAVPLN::SolidObstacle(int& obs_start_ind, int& obs_end_ind,  sensor_msgs::LaserScan laser){  
    //cout<<"SO"<<endl;
    const int sub_obs_width = 30;
    int sub_obs = 0;
    int i = 0;
    double average_x = 0;
    double average_y = 0;
    double angle = 0;
    Eigen::Matrix<double, 2, 1> obs = Eigen::MatrixXd::Zero(2,1);

    //--divido ostacolo in sottogruppi
    if(obs_end_ind - obs_start_ind <= sub_obs_width) sub_obs=1;
    else{
        sub_obs = int (floor((obs_end_ind - obs_start_ind)/sub_obs_width));
    }
    //--

    //--per ogni sottogruppo, considero un solo punto posizionato in (average_x,average_y)
    i = 0;
    average_x = 0;
    average_y = 0;
    angle = 0;
    obs = Eigen::MatrixXd::Zero(2,1);

    _min_obs_dist = 100;    //reset variabile

    if (sub_obs == 1){
        i = obs_start_ind;
        while(i <= obs_end_ind){
            angle = i * laser.angle_increment;  //l'angolo viene coperto da dx a sn
            average_x += laser.ranges[i] * sin(angle);
            average_y += laser.ranges[i] * cos(angle);
            i++;
        }
        obs(0) = double(average_x) / (obs_end_ind - obs_start_ind);
        obs(1) = double(average_y) / (obs_end_ind - obs_start_ind); //passo da laser a NED
        _min_obs_dist = obs.norm();
        //ROS_INFO("OBS [ %f %f]",obs(0),obs(1) );
        _obs_list.push_back(obs);
    }
    else{
        while(sub_obs>0){
            i = obs_start_ind + (sub_obs - 1) * sub_obs_width;
            while(i < (obs_start_ind + sub_obs * sub_obs_width)){
                angle = i * laser.angle_increment;  //l'angolo viene coperto da sn a dx
                average_x += laser.ranges[i] * sin(angle);
                average_y += laser.ranges[i] * cos(angle);
                i++;
            }
            obs(0) = double(average_x) / sub_obs_width;
            obs(1) = double(average_y) / sub_obs_width; //passo da laser a NED
            //ROS_INFO("OBS [ %f %f]",obs(0),obs(1) );
            _obs_list.push_back(obs);

            if(_min_obs_dist > obs.norm()) _min_obs_dist = obs.norm();


            average_x=0;
            average_y=0;
            sub_obs--;
        }
    }
    //--
} 

void MAVPLN::marker_cb( const geometry_msgs::PoseStamped& marker ){

    //ROS_INFO("\nmarker_cb\n");

    if( _takeoff_ok && !(_emp_obs_approaching || _emp_obs_crossing) ){

        // evito di usare il listener di tf perchè sono trasformazioni note e fisse

        /*--
        camera_link aruco -> camera_link
        Anche se aruco prende camera_link come riferimento,
        in realtà restituisce le misure secondo un altro frame:
        è necessario utilizzare R_fake2real_cl per portarsi al 
        camerra_link effettivo
        */

        Eigen::Matrix3d R_fake2real_cl; 
        R_fake2real_cl.row(0) <<  0, 0, 1;
        R_fake2real_cl.row(1) << -1, 0, 0;
        R_fake2real_cl.row(2) <<  0,-1, 0;
        // marker position realtive to camera_link
        Eigen::Vector3d cl_p;
        cl_p(0)=marker.pose.position.x;
        cl_p(1)=marker.pose.position.y;       
        cl_p(2)=marker.pose.position.z;       
        cl_p=R_fake2real_cl*cl_p;
        cl_p(0) = cl_p(0)/1.1;      //calibration
        cl_p(1) = cl_p(1)+0.1;      //calibration
        cl_p(2) = cl_p(2);          //calibration
        //ROS_INFO("cl_p real: [%f %f %f]",cl_p(0),cl_p(1),cl_p(2));
       
        /*
        // marker orientation realtive to camera_link
        tf::Quaternion q_cl( marker.pose.orientation.x, marker.pose.orientation.y , marker.pose.orientation.z ,  marker.pose.orientation.w );
        tf::Matrix3x3 R_cl_tf (q_cl);
        Eigen::Matrix3d R_cl;
        tf::matrixTFToEigen(R_cl_tf, R_cl);
        R_cl= R_fake2real_cl * R_cl;
        */
        //--
        
        //-- camera_link -> worldNED
        Eigen::Matrix3d R_cl2blNED; //rotazione camera_link -> base_linkNED 
        R_cl2blNED.row(0) <<  0,-1, 0;
        R_cl2blNED.row(1) << -1, 0, 0;
        R_cl2blNED.row(2) <<  0, 0,-1;
        Eigen::Vector3d wNED_p, blNED_p;
        blNED_p = R_cl2blNED * cl_p;
        blNED_p(2) -= 0.1; //offset camera lungo asse z 
        wNED_p = _curr_p + blNED_p;

        //ROS_INFO("_curr_p : [%f %f %f]",_curr_p(0),_curr_p(1),_curr_p(2));
        //ROS_INFO("wNED_p : [%f %f %f]",wNED_p(0),wNED_p(1),wNED_p(2));
        //--

        //--  camera_link -> laser_link
        Eigen::Vector3d ll_p;
        ll_p = cl_p;
        ll_p(2)+=0.08; //offset tra i due frame
        //--

        //-- Ricavo posizione centro finestra a partire dalla posizione del marker

        // Posizione centro finestra in worldNED
        _emp_pos_wned = wNED_p;
        _emp_pos_wned(0) += 0.65;
        _emp_pos_wned(1) -= 0.1;
        _emp_pos_wned(2) -= 0.7;
        //ROS_INFO("_emp_pos_wned : [%f %f %f]",_emp_pos_wned(0),_emp_pos_wned(1),_emp_pos_wned(2));
        // Posizione centro finestra in laser frame
        _emp_pos_ll = ll_p;
        //_emp_pos_ll(1) += 0.1; // non lo aggiungo perchè mi interessa il centro rispetto alla faccia frontale
        _emp_pos_ll(1) -= 0.7;
        _emp_pos_ll(2) += 0.6;
        //ROS_INFO("_emp_pos_ll : [%f %f %f]",_emp_pos_ll(0),_emp_pos_ll(1),_emp_pos_ll(2));
        //--


        EmptyObstacle();


    }
  
}

void MAVPLN::EmptyObstacle(){

    double emp_d = sqrt(pow(_emp_pos_ll(0),2) + pow(_emp_pos_ll(1),2));

    if( emp_d > 2){  //empty obstacle lontano dal drone

        _emp_obs_near = false;
        _emp_obs_distant=true;  //altera laser_cb

    }
    else{
        _emp_obs_near = true;   //disattiva completamente la laser_cb
        _emp_obs_distant = false;
    }

}


void MAVPLN::Planning(const Eigen::Matrix<double,3,1>& ATT, const Eigen::Matrix<double,3,1>& REP){

 //custom_msg_pkg::planner_msg
 //    int ctrl_action
 //    geometry_msgs/Vector3 pos
 //    geometry_msgs/Vector3 vel
 //    geometry_msgs/Vector3 acc
 //    float64 yaw
 //    float64 dyaw
 //    float64 ddyaw


    ////cout<<"PLN"<<endl;
    Eigen::Matrix<double,3,1> TF = ATT + REP; 
    
    if(TF.norm()<=0.02) _TF_zero = true;
    else _TF_zero = false;

    if(_TF_zero && TotalPotentialIsNotZero() && !_dest_free){
        LocalMinima(); 
    }

    if(_dest_free) TF = ATT;

    //double acc_sat = 0.6;
    //if(REP.norm()>0 && TF.norm()>acc_sat) TF = (acc_sat/TF.norm())*TF;

    _des_ddp = TF;    
	_des_dp =_curr_dp + Ts * _des_ddp;


    //--Saturazione velocità
    /*
        _des_dp è l'uscita di un integratore e più il drone si avvicina all'ostacolo
        e più aumenta la forza repulsiva, più cresce _des_dp. Quando 
        poi l'ostacolo viene superato e la forza repulsiva si azzera, _des_dp
        resta alto ancora per un poco di tempo, dunque il drone si allontana molto prima
        di arrivare nella posizione desiderata. Accade una sorta di wind-up dell'integratore.
        Aggiungo la saturazione per evitare questo fenomeno.
    */
    double vel_sat = 0.3;
    if(_des_dp.norm()>vel_sat) _des_dp = (vel_sat/_des_dp.norm())*_des_dp;
    //--
    _des_p = _curr_p + Ts * _des_dp;  

    _des_ddyaw = 0;                                                  
    _des_dyaw =0; 
    _des_yaw  =0;        

    custom_msg_pkg::planner_msg p_msg;

    p_msg.ctrl_action = 1;
    p_msg.pos.x = _des_p(0);
    p_msg.pos.y = _des_p(1);
    p_msg.pos.z = _des_p(2);
    p_msg.vel.x = _des_dp(0);
    p_msg.vel.y = _des_dp(1);
    p_msg.vel.z = _des_dp(2);
    p_msg.acc.x = _des_ddp(0);
    p_msg.acc.y = _des_ddp(1);
    p_msg.acc.z = _des_ddp(2);

    p_msg.yaw = _des_yaw;
    p_msg.dyaw = _des_dyaw;
    p_msg.ddyaw = _des_ddyaw;

    _plan_pub.publish(p_msg);


    force_potential_msg.data.resize(8);
    force_potential_msg.data[0]=ATT(0);
    force_potential_msg.data[1]=ATT(1);
    force_potential_msg.data[2]=ATT(2);
    force_potential_msg.data[3]=REP(0);
    force_potential_msg.data[4]=REP(1);
    force_potential_msg.data[5]=REP(2);
    force_potential_msg.data[6]=_att_pot;
    force_potential_msg.data[7]=_rep_pot;
    _for_pot_pub.publish(force_potential_msg);

    //ROS_INFO("\nTOT_F [%f %f %f]",TF(0), TF(1), TF(2));
    //ROS_INFO("TOT_F zero:\t %s ", _TF_zero ? "true" : "false");
    //ROS_INFO("_tot_pot_not_zero:\t %s ", _tot_pot_not_zero ? "true" : "false");
    //ROS_INFO("_dest_free:\t %s ", _dest_free ? "true" : "false");
    //ROS_INFO("\ncurr_p [%f %f %f]",_curr_p(0),_curr_p(1),_curr_p(2));
    //ROS_INFO("\ndes_p [%f %f %f]",_des_p(0),_des_p(1),_des_p(2));
    //ROS_INFO("\ndes_dp [%f %f %f]",_des_dp(0),_des_dp(1),_des_dp(2));
    //ROS_INFO("\ndes_ddp [%f %f %f]",_des_ddp(0),_des_ddp(1),_des_ddp(2));
    //ROS_INFO("\ndes_yaw des_dyaw des_ddyaw [%f %f %f]",_des_yaw,_des_dyaw,_des_ddyaw);

}

Eigen::Matrix<double,3,1> MAVPLN::RepulsiveForce(){
    ////cout<<"RF"<<endl;
   
    const double ROI = 2.0;   //Range of influence of an obstacle: ROI > _al

    Eigen::Matrix<double,3,1> REP_F = Eigen::MatrixXd::Zero(3,1);

    _stop_laser_update = true;  //per evitare che venga aggiornata _obs_list mentre la utilizzo

    
    _rep_pot = 0;
    if(_found_obstacle){

        _found_obstacle = false;
        double obs_dist = 0;
        double force_mod = 0;
        Eigen::Matrix<double, 2, 1> obs =  Eigen::MatrixXd::Zero(2,1);

        //if(rep_gain_p <_K_obs) rep_gain_p+=0.0001;

        for (int i = 0; i != _obs_list.size(); i++){

            obs = _obs_list[i];
            obs_dist = obs.norm();

            if( obs_dist < ROI ){
                
                force_mod = double( _K_obs * ( double(1)/(obs_dist) - double(1)/(ROI) ) ) / pow(obs_dist,2);
                REP_F(0) -= double(force_mod * obs(0)) / obs_dist;  // il meno per dirigerla verso il drone
                REP_F(1) -= double(force_mod * obs(1)) / obs_dist;  // il meno per dirigerla verso il drone

                _rep_pot += 0.5 * _K_obs * pow(double(1)/(obs_dist) - double(1)/(ROI),2);
            
            }
        }
    }

    _stop_laser_update = false;

    ROS_INFO("\nREP_F\t[%f %f %f], norm: %f",REP_F(0),REP_F(1),REP_F(2), REP_F.norm());

    //if(REP_F.norm()>1.0) REP_F=(1.0/REP_F.norm())*REP_F;

    return REP_F;

} 

Eigen::Matrix<double,3,1> MAVPLN::AttractiveForce( const Eigen::Vector3d& destination){
    
    ////cout<<"AF"<<endl;
                           
    Eigen::Matrix<double,3,1> ATT_F = Eigen::MatrixXd::Zero(3,1);

    Eigen::Vector3d p_err = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d dp_err = Eigen::MatrixXd::Zero(3,1);
    double yaw_err = 0;
    double dyaw_err = 0;

    p_err = destination - _curr_p;
    dp_err = - _curr_dp;    //voglio arrivare con velocità nulla

    if( abs(p_err(0))<0.01 && abs(p_err(1))<0.01 && abs(p_err(2))<0.01 ){ _dest_reached = true;}


    yaw_err = - _curr_eta(2);   //voglio arrivare con yaw=0°
    dyaw_err = - _curr_deta(2);

    if(gain_p < _Kp_p) gain_p +=0.1;
    if(_dest_free && gain_p < 1) gain_p +=0.01;
    
    if( p_err.norm() <= 1 ){//aggiungo smorzamento per arrivare con velocità nulla in err=0.
        ATT_F(0) = gain_p * p_err(0) + gain_d * dp_err(0);
        ATT_F(1) = gain_p * p_err(1) + gain_d * dp_err(1);
        ATT_F(2) = gain_p * p_err(2) + gain_d * dp_err(2); 
        if(gain_d <=_Kd_p) gain_d +=0.1;
        _att_pot = 0.5*gain_p*pow(p_err.norm(),2) + 0.5*gain_d*pow(dp_err.norm(),2);
    }
    else{
        ATT_F(0) = (double(gain_p)/p_err.norm()) * p_err(0);
        ATT_F(1) = (double(gain_p)/p_err.norm()) * p_err(1);
        ATT_F(2) = (double(gain_p)/p_err.norm()) * p_err(2); // + (double(_Kd_p)/dp_err.norm()) * dp_err;

        _att_pot = gain_p * p_err.norm();
        
    }
    

    //ROS_INFO("\np_err [%f %f %f]",p_err(0),p_err(1),p_err(2));
    //ROS_INFO("\ndp_err [%f %f %f]",dp_err(0),dp_err(1),dp_err(2));
    //ROS_INFO("\nyaw_err dyaw_err [%f %f ]",yaw_err, dyaw_err);
    ROS_INFO("\nATT_F\t[%f %f %f ], norm: %f",ATT_F(0),ATT_F(1),ATT_F(2), ATT_F.norm());

    return ATT_F;

} 

void MAVPLN::LocalMinima(){

/*
    Se la via è libera, allora trascuro repulsive force e procedo fino all'arrivo,
    altrimenti piazzo un nuovo punto da raggiungere, ad una distanza per la quale 
    sono certo di non avere ostacoli
*/

    if ( (_des_p - _curr_p).norm() <= _min_obs_dist) _dest_free = true;
    else {
        _dest_free = false;

        /*
            il nuovo punto sarà interno alla circonferenza centrata in 
            _curr_p e di raggio inferiore a _min_obs_dist
        */
        double r = 0.5 *_min_obs_dist;
        double a = static_cast <double> (rand()) / static_cast <double> (RAND_MAX); //genero in  [0.0; 1.0]
        if (a < 0 || abs(a)>1 ) a = 0.2; 
        double b = sqrt(1-pow(a,2));
        a=sqrt(a);
        Eigen::Vector3d p;
        p(0) =  _curr_p(0) + a*r;
        p(1) =  _curr_p(1) + b*r;
        p(2) =  _curr_p(2);
        _wp_list.push_back( p ); 
        _wp_index++; 

    }



}

bool MAVPLN::TotalPotentialIsNotZero(){

    if((_att_pot + _rep_pot) > 0) _tot_pot_not_zero = true;
    else _tot_pot_not_zero = false;
    
    return _tot_pot_not_zero;
}




void MAVPLN::WpListManager(){
    // NON CAMBIARE ORDINE IF

    if (! _add_distant && _emp_obs_distant){ROS_INFO("IF 1");
        Eigen::Vector3d p; //worldNED frame
        
        if( (_wp_list.size() == 0) ||
            ((_emp_pos_wned -_curr_p).norm() < (_wp_list.back()-_curr_p).norm())
            ){ //se la finestra è più vicina della destinazione
           ROS_INFO("IF 1_!!!");          
            //--aggiungo punto frontalmente alla finestra, a distanza 1 m
            p(0) =  _emp_pos_wned(0);
            p(1) =  _emp_pos_wned(1) + 1 ;
            p(2) =  _emp_pos_wned(2);
            _wp_list.push_back( p ); 
            // NON incrementare _wp_index
            //--
             _add_distant = true;
        }
         
    }

    if (!_add_near && _emp_obs_near && !_emp_obs_approaching){ ROS_INFO("IF 2");
        Eigen::Vector3d p; //worldNED frame

        if (_add_distant && !_add_near){ // vedo la finestra prima da lontano
            //--rimuovo punto aggiunto quando ero lontano
            _wp_list.pop_back();
            //--
            //--aggiungo punto fronte finestra da vicino (con valore più preciso)
            p(0) =  _emp_pos_wned(0);
            p(1) =  _emp_pos_wned(1) + 1 ;
            p(2) =  _emp_pos_wned(2);
            _wp_list.push_back( p ); 
            //--
            _add_near=true;
            _emp_obs_approaching=true;
            _emp_obs_distant = false;
            _emp_obs_near = false;

        }
        else if (!_add_distant && !_add_near){
            Eigen::Vector3d p; //worldNED frame
        
            if( (_wp_list.size() == 0) ||
                ((_emp_pos_wned -_curr_p).norm() < (_wp_list.back()-_curr_p).norm())
                ){ //se la finestra è più vicina della destinazione
                ROS_INFO("IF _!!!");          
                //--aggiungo punto frontalmente alla finestra, a distanza 1 m
                p(0) =  _emp_pos_wned(0);
                p(1) =  _emp_pos_wned(1) + 1 ;
                p(2) =  _emp_pos_wned(2);
                _wp_list.push_back( p ); 
                // NON incrementare _wp_index
                //--
                _add_distant = true;
                _add_near=true;
                _emp_obs_approaching=true;
                _emp_obs_distant = false;
                _emp_obs_near = false;
            }
        }

    }

    if(_dest_reached && _emp_obs_approaching){ ROS_INFO("IF 3"); //raggiunto punto davanti finestra

        Eigen::Vector3d p; //worldNED frame
        //--rimuovo punto fronte finestra
        _wp_list.pop_back();
        //--
        //--aggiungo punto dietro
        p(0) =  _emp_pos_wned(0); 
        p(1) =  _emp_pos_wned(1) - 0.5;
        p(2) =  _emp_pos_wned(2);
        _wp_list.push_back( p ); 
        //--

        _emp_obs_distant = false;
        _emp_obs_near = false;
        _emp_obs_approaching = false;
        _dest_reached = false;
        _emp_obs_crossing = true;

    }
  
    if(_dest_reached && _emp_obs_crossing){ROS_INFO("IF 4");  //raggiunto punto dietro finestra
        //--rimuovo punto retro finestra
        _wp_list.pop_back();
        _wp_index = _wp_list.size();
        //--


        _emp_obs_distant = false;
        _emp_obs_near = false;
        _emp_obs_approaching = false;
        _emp_obs_crossing = false;
        _dest_reached = false;
        _dest_free = false;
        _add_distant = false;
        _add_near = false;

        if(_wp_index == 0 || _wp_list.size()==0) _landing = true;

        rep_gain_p=0;
        gain_d=0.1;
        gain_p=0;
    }
  
    if(_dest_reached){ ROS_INFO("IF 5");//raggiunta una delle destinazioni settate all'avvio del sistema

        _dest_reached = false;
        _dest_free = false;
        _wp_list.pop_back();
        _wp_index--;
        if( _wp_index == 0 &&
            !(_emp_obs_distant || _emp_obs_near)
          ){
            _landing = true;
        }            

        rep_gain_p=0;
        gain_d=0.1;
        gain_p=0;
    }
        
    for (int i = 0; i != _wp_list.size(); i++) ROS_INFO("_wp_list[%d]\t [%f %f %f]",i,_wp_list[i][0],_wp_list[i][1],_wp_list[i][2]);
    ROS_INFO("_emp_obs_distant %s",_emp_obs_distant ? "true" : "false");
    ROS_INFO("_emp_obs_near %s",_emp_obs_near ? "true" : "false");
    //ROS_INFO("_add_distant %s",_add_distant ? "true" : "false");
    //ROS_INFO("_add_near %s",_add_near ? "true" : "false");
    ROS_INFO("_emp_obs_approaching %s",_emp_obs_approaching ? "true" : "false");
    ROS_INFO("_emp_obs_crossing %s",_emp_obs_crossing ? "true" : "false");
    ROS_INFO("_dest_free %s",_dest_free ? "true" : "false");
    ROS_INFO("_dest_reached %s",_dest_reached ? "true" : "false");
    ROS_INFO("_wp_index %d\t_wp_size %d",_wp_index, int(_wp_list.size()));

}




void MAVPLN::TakeOff(){


    Eigen::Vector3d p_err = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d takeoff_goal = Eigen::MatrixXd::Zero(3,1);
    
    custom_msg_pkg::planner_msg p_msg;

    takeoff_goal(0)=_curr_p(0);
    takeoff_goal(1)=_curr_p(1);
    takeoff_goal(2)=-1;

    p_msg.ctrl_action = 0;
    p_msg.pos.x = takeoff_goal(0);
    p_msg.pos.y = takeoff_goal(1);
    p_msg.pos.z = takeoff_goal(2);
    p_msg.vel.x = 0;
    p_msg.vel.y = 0;
    p_msg.vel.z = 0;
    p_msg.acc.x = 0;
    p_msg.acc.y = 0;
    p_msg.acc.z = 0;

    p_msg.yaw = _curr_eta(2);
    p_msg.dyaw = 0;
    p_msg.ddyaw = 0;

    ros::Rate r(fs);
    double tc = 0;

    while ( ros::ok() && !_takeoff_ok) {

        ROS_INFO("TAKE OFF:\t %s ", !_takeoff_ok ? "true" : "false");

        _plan_pub.publish(p_msg);

        p_err = takeoff_goal - _curr_p;
        
        ROS_INFO("p_err: [ %f %f %f ] ", p_err(0), p_err(1), p_err(2));

        while( ros::ok() && !_takeoff_ok && abs(p_err(0))<0.01 && abs(p_err(1))<0.01 && abs(p_err(2))<0.01 ){ 

            if(tc>1)_takeoff_ok = true;
            tc+=Ts;
            p_err = takeoff_goal - _curr_p;
            
            ROS_INFO("tc:  %f\t p_err: [ %f %f %f ] ", tc, p_err(0), p_err(1), p_err(2));
            
            r.sleep();
        }

        if(!_takeoff_ok) tc=0;        

        r.sleep();
    }

    _des_ddp = Eigen::MatrixXd::Zero(3,1);
    _des_dp  = _curr_dp;
    _des_p   = _curr_p;

    _des_ddyaw = 0;
    _des_dyaw  = _curr_deta(2);
    _des_yaw   = _curr_eta(2);

}

void MAVPLN::Landing(){



    bool landing_ok = false;

    Eigen::Vector3d p_err = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d landing_goal = Eigen::MatrixXd::Zero(3,1);
    
    custom_msg_pkg::planner_msg p_msg;

    landing_goal(0)=_curr_p(0);
    landing_goal(1)=_curr_p(1);
    landing_goal(2)=-0.35;

    p_msg.ctrl_action = 2;
    p_msg.pos.x = landing_goal(0);
    p_msg.pos.y = landing_goal(1);
    p_msg.pos.z = landing_goal(2);
    p_msg.vel.x = 0;
    p_msg.vel.y = 0;
    p_msg.vel.z = 0;
    p_msg.acc.x = 0;
    p_msg.acc.y = 0;
    p_msg.acc.z = 0;

    p_msg.yaw = _curr_eta(2);
    p_msg.dyaw = 0;
    p_msg.ddyaw = 0;

    ros::Rate r(fs);

    double tc = 0;

    while ( ros::ok() && !landing_ok) {


        ROS_INFO("LANDING:\t %s ", !landing_ok ? "true" : "false");

        _plan_pub.publish(p_msg);

        p_err = landing_goal - _curr_p;

        while( ros::ok() && !landing_ok && abs(p_err(0))<0.01 && abs(p_err(1))<0.01 && abs(p_err(2))<0.01 ){ 

            if(tc>1)landing_ok = true;
            tc+=Ts;
            p_err = landing_goal - _curr_p;
            r.sleep();
        }

        if(!landing_ok) tc=0;  

        r.sleep();
    }

    p_msg.ctrl_action = 3;  //spegnere motori
    p_msg.pos.x = landing_goal(0);
    p_msg.pos.y = landing_goal(1);
    p_msg.pos.z = landing_goal(2);
    p_msg.vel.x = 0;
    p_msg.vel.y = 0;
    p_msg.vel.z = 0;
    p_msg.acc.x = 0;
    p_msg.acc.y = 0;
    p_msg.acc.z = 0;

    p_msg.yaw = _curr_eta(2);
    p_msg.dyaw = 0;
    p_msg.ddyaw = 0;

    _plan_pub.publish(p_msg);
    ROS_INFO("STOP ROTORS!");

}





void MAVPLN::pln_loop(){


    while(!(_first_laser && _first_odom)) sleep(0.1);

    TakeOff();

    ros::Rate r(fs);

    while ( ros::ok() && !_landing) {

        Planning ( AttractiveForce(_wp_list.back()) , RepulsiveForce() );   //NED frame
        
        WpListManager();
        
        r.sleep();
    }

    Landing();

}

void MAVPLN::run() {
  
    
    boost::thread pln_loop_t( &MAVPLN::pln_loop, this );

    ros::spin();
}




int main(int argc, char** argv) {
    ros::init( argc, argv, "MAVPLN_node");

    MAVPLN pln;

    pln.run();

    return 0;
}
