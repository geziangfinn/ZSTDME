#include "ctsdb.h"

void CTSDB::showCTSdbInfo()
{
    cout.setf(ios::fixed, ios::floatfield);
    cout<<padding<<" Database summary "<<padding<<endl;
    cout<<"Sink count: "<<dbSinks.size()<<endl;
    cout<<"Blockage count: "<<dbBlockages.size()<<endl;
    
    // for(Sink sink:dbSinks)
    // {
    //     cout<<"sink: "<<sink.x<<" "<<sink.y<<" "<<sink.capacitance<<endl;
    // }
    // cout<<"========================"<<endl;
    // for(Blockage blockage:dbBlockages)
    // {
    //     cout<<blockage.ll.x<<" "<<blockage.ll.y<<" "<<blockage.ur.x<<" "<<blockage.ur.y<<endl;
    // }
}