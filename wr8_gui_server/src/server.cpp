
#include <gui_server.hpp>

int main(int argc, char **argv)
{
	/* ROS routine */
	ros::init(argc, argv, "gui_server");

	/* Qt server routine */	
	QCoreApplication app(argc, argv);
	qRegisterMetaType<DataPackage>();

	SVServer server;
	server.start( QHostAddress("0.0.0.0"), 5556 );

	ROSServer ros_server(&server);
	ros_server.start();

	QObject::connect( &ros_server, SIGNAL(rosShutdown()), &app, SLOT(quit()), Qt::QueuedConnection );

    return app.exec();
}
