//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package inet.examples.myexamples.LogNormalGridTestTX;

// numOfHosts: 10
// parametric: true
// static:     false

import inet.environment.common.PhysicalEnvironment;
import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.inet.AdhocHost;
import inet.node.inet.USVHost;
//import inet.node.inet.WirelessHost;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;

network LogNormalGridTestTX
{
    parameters:
        int numHosts;
    submodules:
        tower: AdhocHost {
            @display("p=20,20;i=device/antennatower");
        }

        host[numHosts]: USVHost {
            parameters:
                @display("r=,,#707070");
        }

        radioMedium: Ieee80211ScalarRadioMedium {
            parameters:
                @display("p=178,19");
        }

        configurator: IPv4NetworkConfigurator {
            @display("p=113,20");
        }
        environment: PhysicalEnvironment {
            parameters:
                @display("p=254,20");
        }
}
