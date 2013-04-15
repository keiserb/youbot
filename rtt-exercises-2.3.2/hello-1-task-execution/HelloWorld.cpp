
/**
 * @file HelloWorld.cpp
 * This file demonstratess the Orocos TaskContext execution with
 * a 'hello world' example.
 */

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

#include <ocl/OCL.hpp>
#include <ocl/TaskBrowser.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;

/**
 * Exercise 1: Read Orocos Component Builder's Manual, Chap 2 sect 3 and 3.1
 *
 * First, compile and run this application and try to start the component
 * from the TaskBrowser console. How often is updateHook() executed ? Why ?
 *
 * Tip: In order to find out which functions this component has, type 'ls', and
 * for detailed information, type 'help this' (i.e. print the interface of the 'this'
 * task object).
 *
 * Next, Set the period of the component in configureHook to 0.5 seconds and
 * make start() succeed when the period of the component indeed equals 0.5 seconds.
 *
 * Next, add functions which use the log(Info) construct to display
 * a notice when the configureHook(), startHook(), stopHook() and cleanupHook()
 * are executed. (Note: not all these functions return a bool!)
 *
 * Recompile and restart this application and try to configure, start, stop
 * and cleanup the component.
 *
 * Optional: Let the Hello component be created in the 'PreOperational' mode.
 * What effect does this have on the acceptance of the start() method ?
 * Optional: Replace the Activity with a SlaveActivity. What are
 * the effects of trigger and update in comparison with the other activity types ?
 * Optional: Replace the Activity with a SequentialActivity. What are
 * the effects of trigger and update in comparison with the other activity types ?
 */
namespace Example
{

    /**
     * Every component inherits from the 'TaskContext' class.  This base
     * class allow a user to add a primitive to the interface and contain
     * an ExecutionEngine which executes application code.
     */
    class Hello
        : public TaskContext
    {
    public:
        /**
         * This example sets the interface up in the Constructor
         * of the component.
         */
        Hello(std::string name)
            : RTT::TaskContext(name)
        {
        }

        void updateHook()
        {
        	log(Info) << "Update !" <<endlog();
        }
        bool configureHook()
        {
        	return true;
        }
    };
}

using namespace Example;

int ORO_main(int argc, char** argv)
{
    Logger::In in("main()");

    // Set log level more verbose than default,
    // such that we can see output :
    if ( log().getLogLevel() < Logger::Info ) {
        log().setLogLevel( Logger::Info );
        log(Info) << argv[0] << " manually raises LogLevel to 'Info' (5). See also file 'orocos.log'."<<endlog();
    }

    log(Info) << "**** Creating the 'Hello' component ****" <<endlog();
    // Create the task:
    Hello hello("Hello");

    log(Info) << "**** Starting the TaskBrowser       ****" <<endlog();
    // Switch to user-interactive mode.
    TaskBrowser browser( &hello );

    // Accept user commands from console.
    browser.loop();

    return 0;
}
