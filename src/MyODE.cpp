#include "MyODE.h"
#include <Eigen/Dense>
#include <fstream>
#include <random>
#include <chrono>

void MyODE::init_params(){
    // g0 = 4.2118e-11; // orbit 1-3
	g0 = 3.1244e-11; // orbit 4
	C1 = 2.0;
	C2 = 0.0;
	C3 = 0.0;
    // unit_acc = 2.55251e-12; // orbit 1-3
	unit_acc = 1.45435e-12; // orbit 4
    // unit_length = 180.404; // orbit 1-3
	unit_length = 52.4409; // orbit 4
    // std::cout << "ode.init_params()\n\tg0: " << g0 
    //           << "\n\tunit_acc: " << unit_acc
    //           << "\n\tunit_length: " << unit_length
    //           << "\n";
	u_data.clear();

    W_intensity = 0.0;
}


void MyODE::set_W_intenstiy(const double W){
    W_intensity = W;
}


double MyODE::get_W_intensity(){
    return W_intensity;
}

Eigen::VectorXd MyODE::get_dxdt(double t, Eigen::VectorXd x, const std::vector<double> u){
	double r1 = x[0]; double r2 = x[1]; double r3 = x[2];
    double v1 = x[3]; double v2 = x[4]; double v3 = x[5];

    double r = pow(r1*r1 + r2*r2 + r3*r3, 0.5);
	std::vector<double> a_SRP = get_a_SRP(u[0], u[1]);
    double ax = a_SRP[0]/unit_acc;
    double ay = a_SRP[1]/unit_acc;
    double az = a_SRP[2]/unit_acc;
    //std::cout << "a_SRP: " << ax << " " << ay << " " << az <<  "\n";

    // additive white guassian noise
    Eigen::VectorXd w_vector = generate_process_noise(3);
    // log_vector(w_vector);
    ax = ax + w_vector[0]; ay = ay + w_vector[0]; az = az + w_vector[0];

    const int size = x.size();
    Eigen::VectorXd dxdt(size);
    dxdt[0] = v1;
    dxdt[1] = v2;
    dxdt[2] = v3;
    dxdt[3] =  2*v2 + 3*r1 - r1/pow(r,3) + ax;
    dxdt[4] = -2*v1 - r2/pow(r,3) + ay;
    dxdt[5] = -r3 - r3/pow(r,3) + az;
    return dxdt;
}

std::vector<double> MyODE::get_a_SRP(double u1, double u2){
    std::vector<double> a_SRP;
    a_SRP.push_back( g0*cos(u1)*( C1*pow(cos(u1),2) + C2*cos(u1) + C3 ) );
    a_SRP.push_back( g0*cos(u1)*( -(C1*cos(u1)+C2)*sin(u1)*sin(u2) ) );
    a_SRP.push_back( g0*cos(u1)*( -(C1*cos(u1)+C2)*sin(u1)*cos(u2) ) );
    return a_SRP;
}

// Runge-Kutta solver function
std::vector<Eigen::VectorXd> MyODE::rungeKutta(const double t0, const Eigen::VectorXd x0, const std::vector<double> u, 
                                               const double h,  const double t_end){

    // std::cout << "Runge-Kutta: t_end: " << t_end << "\n";
    const int size = x0.size();
    std::vector<Eigen::VectorXd> state_traj;
    double t = t0;
    Eigen::VectorXd x(size);
    x = x0;
    state_traj.push_back(x);

    int k = 0;
    while (t < t_end) {
        // std::cout << k << "\n";
        if(isTerminal(x)){
            return state_traj;
        }

        // init u at time k
        std::vector<double> uk;
        // a. get u control from u.csv data
        if(u_data.size() > 0){
            // std::cout << "use data control\n";
            if(k < u_data.size()){
                uk = u_data[k];
            }
            // exceeding the u.csv data size
            else{
                return state_traj;
            }
        }
        // b. prespecified u signal
        else{
            // std::cout << "use sample control\n";
            uk = u;
        }
        // std::cout << k << " " << t << "\n"; // log_vector(uk);

        k = k + 1;

        Eigen::VectorXd w1(size);
        Eigen::VectorXd w2(size);
        Eigen::VectorXd w3(size);
        Eigen::VectorXd w4(size);

        w1 = get_dxdt(t, x, uk);
        w2 = get_dxdt(t, x + 0.5*h*w1, uk);
        w3 = get_dxdt(t, x + 0.5*h*w2, uk);
        w4 = get_dxdt(t, x + 1.0*h*w3, uk);

        x = x + h*(w1 + 2*w2 + 2*w3 + w4)/6;
        t = t + h;
        state_traj.push_back(x);
    }

    return state_traj;
}

bool MyODE::isTerminal(const Eigen::VectorXd& s){
    double km2m = 1000.0;
    // std::cout << "ode isterminal\n"; log_vector(s);
    double x =  abs(s[0]*unit_length*km2m); //std::cout << "get x\n";
    double y =  abs(s[1]*unit_length*km2m);
    double z =  abs(s[2]*unit_length*km2m);
    double r = pow(x*x + y*y + z*z, 0.5); //std::cout << "ode isterminal r: " << r << " "; log_vector(s);
    if(r <= 250.0){
        return true;
    }
    return false;
}

void MyODE::validate(const Eigen::VectorXd x0, const std::string ufilename, const std::string filename){
// To validata: make sure to change
// x0 (initial states)
// t_end (final simulation time)
// beta (the non-dimensional solar sail acceleration)
// mu (asteroid gravity constant)
// d (asteroid to sun distance)

	const double t_start = 0.0;
	double t_end = 0.5;
	const double tstep_Traj = 0.0001;

	// (Alternatively) read u data
	if(ufilename != ""){
		read_u_control(ufilename);
        t_end = u_data.size();
	}
	std::vector<double> u_control;
	const double u1 = 0;
	const double u2 = 0;
	u_control.push_back(u1);
	u_control.push_back(u2);
	
	std::vector<Eigen::VectorXd> x_traj;
	x_traj = rungeKutta(t_start, x0, u_control, tstep_Traj, t_end);
	log_vector(x_traj[x_traj.size()-1]);
	
	write_traj_csv(x_traj, filename);
}

void MyODE::write_traj_csv(const std::vector<Eigen::VectorXd>& data, const std::string filename){
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file for writing." << std::endl;
    }
    // Set a high precision for the output stream
    // outputFile << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& vectorData : data) {
        for (int i = 0; i < vectorData.size(); ++i) {
            outputFile << vectorData(i);
            if (i < vectorData.size() - 1) {
                outputFile << ',';
            }
        }
        outputFile << std::endl;
    }
    outputFile.close();
    std::cout << "Save file to: " << filename << "\n";
}

void MyODE::read_u_control(const std::string csv_file){
	// Open the .csv file
	std::ifstream file(csv_file);

	if (!file.is_open()) {
		std::cerr << "Failed to open the .csv file." << std::endl;
	}

	std::string line;
	while (std::getline(file, line)) {
		std::vector<double> row;
		std::istringstream ss(line);
		std::string cell;

		while (std::getline(ss, cell, ',')) {
			try {
				double value = std::stod(cell);
				row.push_back(value);
			} catch (const std::invalid_argument& e) {
				std::cerr << "Error parsing a non-numeric value: " << cell << std::endl;
			}
		}
		u_data.push_back(row);
	}

	// Close the .csv file
	file.close();
}

void MyODE::log_vector(const Eigen::VectorXd v){
	std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}

void MyODE::log_vector(const std::vector<double> v){
	std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
    }
	std::cout << "\n";
}


std::vector<double> MyODE::sphere_to_cartesian(double r, double theta, double phi){
    std::vector<double> cartesian;
    cartesian.push_back( r * sin(phi) * cos(theta) );
    cartesian.push_back( r * sin(phi) * sin(theta) );
    cartesian.push_back( r * cos(phi) );
    return cartesian;
}


Eigen::VectorXd MyODE::generate_process_noise(const int dimension){
    // assuming guassian additive white noise
    if(W_intensity == 0.0){
        return Eigen::VectorXd::Zero(dimension);
    }
    else{
        // Seed the random number generator with the current time
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(0.0, W_intensity);

        Eigen::VectorXd w_vector(dimension);
        for(int i=0; i<dimension; i++){
            w_vector[i] = distribution(generator);
        }
        return w_vector;
    }
}