#include "plan.hh"
#include <iostream>

using namespace std;
using namespace Nav;


void Plan::removeCorridor(int idx)
{
    corridors.erase(corridors.begin() + idx);
    for (corridor_iterator corridor = corridors.begin(); corridor != corridors.end(); ++corridor)
    {
        Corridor::Connections& connections = corridor->connections;
        Corridor::connection_iterator it = connections.begin();
        while (it != connections.end())
        {
            if (it->get<1>() == idx)
                connections.erase(it++);
            else
            {
                if (it->get<1>() > idx)
                    it->get<1>()--;
                ++it;
            }
        }
    }
}

ostream& Nav::operator << (ostream& io, Plan const& plan)
{
    for (vector<Corridor>::const_iterator it = plan.corridors.begin(); it != plan.corridors.end(); ++it)
    {
        io << "\n==== Corridor " << it - plan.corridors.begin() << "====\n";
        io << *it << endl;
    }
    return io;
}

