#include <QtGui>
#include "GainsTuningWidget.h"


int main( int argc, char* argv[] ) { 

    QApplication app( argc, argv );
    
    GainsTuningWidget gw;
    gw.show();
    app.connect( &app, SIGNAL(lastWindowClosed()),
		 &app, SLOT(quit()) );

    int result = app.exec();

    return result;
}
