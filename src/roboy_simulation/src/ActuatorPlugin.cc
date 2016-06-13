#include "ActuatorPlugin.hh"

using namespace gazebo;

//Register plugin with this simulator
GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin);


ActuatorPlugin::Motor _motor;
ActuatorPlugin::Gear _gear;
ActuatorPlugin::Spindle _spindle;

float tendonForce;
float efficiency;

typedef std::vector< double > state_type;
state_type x(2);
boost::numeric::odeint::runge_kutta4< state_type > stepper;

// std::chrono::high_resolution_clock::time_point t2;
// std::chrono::high_resolution_clock::time_point t1; 
// common::Time sim_t1;
// common::Time sim_t2;

ActuatorPlugin::ActuatorPlugin()
{

}


float ActuatorPlugin::EfficiencyApproximation()
{
	float param1 = 0.1; // defines steepness of the approxiamtion
	float param2 = 0; // defines zero crossing of the approxiamtion
	// float tmp = tanh(-param1 * (-_spindle.angVel) * _motor.current - param2);
	// gzdbg << tmp << "\t" << _spindle.angVel << "\t" << _motor.current << "\n";
	return _gear.efficiency + (1/_gear.efficiency - _gear.efficiency)*(0.5*(tanh(-param1 * _spindle.angVel * _motor.current - param2) +1));
}


void DiffModel( const state_type &x , state_type &dxdt , const double /* t */ )
{
    //x[0] - motor electric current
    //x[1] - spindle angular velocity
	float totalIM = _motor.inertiaMoment + _gear.inertiaMoment; // total moment of inertia
	// gzdbg << "inductance: " << _motor.conductance << "\n";
	// gzdbg << "resistance: " << _motor.resistance << "\n";
	// gzdbg << "BEMFConst: " << _motor.BEMFConst << "\n";
	// gzdbg << "ratio: " << _gear.ratio << "\n";
    dxdt[0] = 1/_motor.inductance * (-_motor.resistance * x[0] -_motor.BEMFConst * _gear.ratio * x[1] + _motor.voltage);
    dxdt[1] = _motor.torqueConst * x[0] / (_gear.ratio * totalIM) - _spindle.radius * tendonForce / (_gear.ratio * _gear.ratio * totalIM * efficiency);
}


void ActuatorPlugin::ComputeDynamics()
{
	using namespace std;
    using namespace boost::numeric::odeint;

    //integrate_observ
    vector<state_type> x_vec;
    vector<double> times;


    runge_kutta4< state_type > stepper;
    
    //stepper.do_step( harmonic_oscillator , x , t , dt );
	//gzbdbg << "current: " << x[0] << "\t" << "velocity: " << x[1] << "\n";
	// size_t steps = integrate( DiffModel ,
	//         x , 0.0 , 10.0 , 0.1 ,
	//         push_back_state_and_time( x_vec , times ) );

	//     /* output */
	//     for( size_t i=0; i<=steps; i++ )
	//     {
	//         gzmsg << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
	//     }
}

void ActuatorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	//Store pointer to the model
	this->model = _parent;

	//Get the parameters from SDF
	gzmsg << "Reading values from SDF" << std::endl;

	if (_sdf->HasElement("motor"))
	{	
		sdf::ElementPtr motorElement = _sdf->GetElement("motor");

		if (//!motorElement->HasElement("electric_current") || 
			!motorElement->HasElement("torque_constant") ||
			!motorElement->HasElement("BEMF_constant") ||
			!motorElement->HasElement("inductance") ||
			!motorElement->HasElement("resistance") ||
			!motorElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}
		
		// if (motorElement->HasElement("electric_current"))
		// {
		// 	//float current;	
		// 	//motor.eCurrent = motorElement->Get<float>("electric_current");	
		// }

		if (motorElement->HasElement("torque_constant"))
		{	
			_motor.torqueConst = motorElement->Get<float>("torque_constant");	
			gzdbg << "torque_constant " <<_motor.torqueConst<< "\n";
		}

		if (motorElement->HasElement("bemf_constant"))
		{
			_motor.BEMFConst = motorElement->Get<float>("bemf_constant");	
			gzdbg << "bemf_constant " <<_motor.BEMFConst<< "\n";
		}

		if (motorElement->HasElement("inductance"))
		{	
			_motor.inductance = motorElement->Get<float>("inductance");	
			gzdbg << "inductance " <<_motor.inductance<< "\n";
		}

		if (motorElement->HasElement("resistance"))
		{	
			_motor.resistance = motorElement->Get<float>("resistance");	
			gzdbg << "resistance " <<_motor.resistance<< "\n";
		}

		if (motorElement->HasElement("inertiaMoment"))
		{	
			_motor.inertiaMoment = motorElement->Get<float>("inertiaMoment");	
			gzdbg << "inertia " <<_motor.inertiaMoment<< "\n";
		}
	}

	if (_sdf->HasElement("gear"))
	{	
		sdf::ElementPtr gearElement = _sdf->GetElement("gear");

		if (!gearElement->HasElement("ratio") ||
			!gearElement->HasElement("efficiency") ||
			!gearElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for gear model";
		}
		
		if (gearElement->HasElement("ratio"))
		{	
			_gear.ratio = gearElement->Get<float>("ratio");	
			gzdbg << "ratio " <<_gear.ratio<< "\n";
		}

		if (gearElement->HasElement("efficiency"))
		{
			_gear.efficiency = gearElement->Get<float>("efficiency");	
			gzdbg << "efficiency " <<_gear.efficiency<< "\n";
		}

		if (gearElement->HasElement("inertiaMoment"))
		{	
			_gear.inertiaMoment = gearElement->Get<float>("inertiaMoment");	
			gzdbg << "inertia " <<_gear.inertiaMoment<< "\n";
		}
	}


	if (_sdf->HasElement("spindle"))
	{	
		sdf::ElementPtr spindleElement = _sdf->GetElement("spindle");

		if (!spindleElement->HasElement("radius"))

		{
			gzwarn << "Invalid SDF: Missing required elements for spindle model";
		}

		// if (spindleElement->HasElement("angular_velocity"))
		// {
		// 	_spindle.angVel = spindleElement->Get<float>("angular_velocity");	
		// }

		if (spindleElement->HasElement("radius"))
		{	
			_spindle.radius = spindleElement->Get<float>("radius");	
			gzdbg << "radius " <<_spindle.radius<< "\n";
		}
	}
	this->connection = event::Events::ConnectWorldUpdateBegin(
         			 boost::bind(&ActuatorPlugin::OnUpdate, this));
}

void ActuatorPlugin::Init()
{
	//state initialization
    x[0] = 0.0; // start at i=0.0, w_g=0.0
    x[1] = 0.0;
    // t1 = std::chrono::high_resolution_clock::now();
    // sim_t1 = this->model->GetWorld()->GetSimTime();
    
}

void ActuatorPlugin::OnUpdate()
{
	
	//compute the step time
	common::Time currTime = this->model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
 	this->prevUpdateTime = currTime;

 	// if (currTime<10000)
 	// {
	 	_motor.current = x[0];
	 	_spindle.angVel = x[1];

	 	//TODO: read voltage from ROS callback
	 	//TODO: get elastic force from tendon plugin
	 // 	if ((int) currTime.Double() % 1000000 == 0)
		// {
		// 	float cur;
		// 	std::cout << "Motor's voltage:";
		// 	std::cin >> _motor.voltage;
	 //        std::cout << "Tendon force:";
		// 	std::cin >> tendonForce;
		// }

		_motor.voltage = 5.0;
		tendonForce = 0.0;

		ofstream logfile;
	    logfile.open("log_torque_rpm.txt", ios::ate | ios::app);

		//calculate the approximation of gear's efficiency
		efficiency = EfficiencyApproximation(); 
		logfile << x[0]*_motor.torqueConst << "\t" << x[1] << "\n";
		//do 1 step of integration of DiffModel() at current time
	    
	    stepper.do_step(DiffModel, x, currTime.Double(), stepTime.Double());
    // }
  	// else
  	// {
  	// 	t2 = std::chrono::high_resolution_clock::now();
  	// 	sim_t2 = this->model->GetWorld()->GetSimTime();
  	// 	auto duration = t2 - t1;
  	// 	auto sim_duration = sim_t2 - sim_t1;
  	// 	gzmsg << std::to_string(&duration) << "<- real time" << "/n";
  	// 	gzmsg << std::to_string(&sim_duration) << "<- simulated time" << "/n";
  	// }
    if ((int) currTime.Double() % 100000 == 0)
	{
		gzdbg << "current: " 
			  << x[0] << "\t" 
			  << "velocity: " 
			  << x[1] << "\n"
			  << "EfficiencyApproximation: "
			  << efficiency;
	
	}
	
	// logfile.close();


}
