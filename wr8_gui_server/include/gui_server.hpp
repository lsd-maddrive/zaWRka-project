#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <svserver.h>

#include <QCoreApplication>
#include <QObject>
#include <QTimer>

class ROSServer : public QThread
{
    Q_OBJECT

    SVServer *m_server;
    QTimer   highSendTmr;
    QTimer   lowSendTmr;

    int32_t m_encoderRotations;
    int32_t m_steeringAngleDeg;

    MapPackage m_map;

    ros::Publisher m_ctrl_pub;

public:
    ROSServer(SVServer *server) :
        m_encoderRotations( 0 ), m_steeringAngleDeg( 0 )
    {
        m_server = server;

        m_map = MapPackage({{0, 0, 0, 0, 0, 0, 0},
                            {0, 1, 0, 1, 0, 1, 0},
                            {0, 1, 0, 1, 0, 0, 0},
                            {0, 0, 0, 0, 0, 1, 1},
                            {0, 1, 0, 1, 0, 1, 1},
                            {0, 1, 0, 1, 0, 0, 0},
                            {0, 1, 0, 0, 0, 1, 1}});

        QObject::connect( server, SIGNAL(signalControl(ControlPackage const&)), this, SLOT(cmdCb(ControlPackage const&)) );
        
        QObject::connect( server, &SVServer::signalNewConnection, this, [this] {
            m_server->slotSendMap( m_map );
        });

        QObject::connect( &highSendTmr, &QTimer::timeout, this, [this] {
            HighFreqDataPackage send_pkg = HighFreqDataPackage();
            send_pkg.m_encoderValue = m_encoderRotations;
            send_pkg.m_steeringAngle = m_steeringAngleDeg;

            m_server->slotSendHighFreqData( send_pkg );
        });

        QObject::connect( &lowSendTmr, &QTimer::timeout, this, [this] {
            LowFreqDataPackage send_pkg = LowFreqDataPackage( LowFreqDataPackage::State::WAIT );

            m_server->slotSendLowFreqData( send_pkg );
        });
        
        highSendTmr.start(100);
        lowSendTmr.start(1000);
    }

    void encoderRawCb(const std_msgs::Int32& msg)
    {
        ROS_INFO("Received encoder data: %d", msg.data);
        m_encoderRotations = msg.data;
    }

    void steerCb(const std_msgs::Float32& msg)
    {
        ROS_INFO("Received steer data: %g", msg.data);
        m_steeringAngleDeg = msg.data;
    }

    void getCarPosition()
    {

    }

    void run()
    {
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("steering", 10, &ROSServer::steerCb, this);
        ros::Subscriber sub1 = n.subscribe("encoder_raw", 10, &ROSServer::encoderRawCb, this);
        
        m_ctrl_pub = n.advertise<geometry_msgs::Twist>("cmd", 10);

        while ( ros::ok() )
        {
            ros::spinOnce();
        }

        Q_EMIT rosShutdown();
    }

private Q_SLOTS:
    void cmdCb(const ControlPackage& data)
    {
        geometry_msgs::Twist msg;

        msg.linear.x = data.yAxis * 0.5;
        msg.angular.y = data.xAxis * 25.0 * M_PI / 180;

        m_ctrl_pub.publish( msg );
    }

Q_SIGNALS:
    void rosShutdown();
    void sendDataHigh(HighFreqDataPackage const& data);
    void sendDataLow(LowFreqDataPackage const& data);
};
