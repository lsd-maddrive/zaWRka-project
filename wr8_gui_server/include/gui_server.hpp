#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <svserver.h>

#include <QCoreApplication>
#include <QObject>
#include <QTimer>

class ROSServer : public QThread
{
	Q_OBJECT

	SVServer *m_server;
	QTimer	 sendTmr;

	DataPackage	m_dataPkg;

public:
    ROSServer(SVServer *server)
    {
    	m_server = server;
    	m_dataPkg = DataPackage( DataPackage::State::WAIT );

    	QObject::connect( this, SIGNAL(sendData(DataPackage const&)), server, SLOT(sendData(DataPackage const&)) );
    	
    	QObject::connect( &sendTmr, SIGNAL(timeout()), this, SLOT(sendTmrHandler()) );
    	
    	sendTmr.start(200);
    }

	void encoderRawCb(const std_msgs::Int32& msg)
	{
		ROS_INFO("Received encoder data: %d", msg.data);
		m_dataPkg.m_encoderValue = msg.data;
	}

	void steerCb(const std_msgs::Float32& msg)
	{
		ROS_INFO("Received steer data: %g", msg.data);
		m_dataPkg.m_steeringAngle = msg.data;
	}

	void run()
	{
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("steering", 1000, &ROSServer::steerCb, this);
		ros::Subscriber sub1 = n.subscribe("encoder_raw", 1000, &ROSServer::encoderRawCb, this);
		
		while ( ros::ok() )
		{
			ros::spinOnce();
		}

		Q_EMIT rosShutdown();
	}

private Q_SLOTS:
	void sendTmrHandler()
    {
		Q_EMIT sendData( m_dataPkg );
    }

Q_SIGNALS:
    void rosShutdown();
    void sendData(DataPackage const& data);
};
