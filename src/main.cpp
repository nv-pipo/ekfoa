
#include <boost/thread.hpp>   // boost::thread

#include "gui.hpp"
#include "ekfoa.hpp"

int main(int argc, char** argv){

	//Start a thread for the Extended Kalman Filter:
	EKFOA ekfoa;
    boost::thread ekfoa_thread (&EKFOA::start, &ekfoa);

	//initialize the OpenGL gui:
	Gui::init();

	bool keep_going = true;

    while (keep_going){
    	keep_going = Gui::redraw();
    }

    ekfoa_thread.join();

	Gui::release();
}
