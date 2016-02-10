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

package inet.applications.broadcastapp;

//
// TODO auto-generated type
//
simple USVControl
{
    double defaultRepulsiveWeigth = default(10); 	// define the weigth parameter in the repulsive force calculation
    double desiredWeigthRatio = default(0.05);		// define the desired ratio of the weigth to calculate the distance from 2 points
    bool pathLossMapAvailable = default(true);		// define if the control module has the pathloss map available
    double checkScanTimeStep @unit(s) = default (3s);	// time to check the scan method
    double scanningTime @unit(s) = default (1s);		// time needed to permirm a scanning
    double scanPowerThreshold @unit(dBm) = default (-90dBm);	// thresholt to decide if the channel is free or not
    double sizeOfScenaioReportCells @unit(m) = default(20m);		// size of the cell to make report on channel availability
    double alphaOffsetDiffCell = default(1.5);		// difference of alpha to consider as a different cell
    string outputCellsScanReport;							// output file name for the cells report
    double sigmaMultiplierInTxRangeCalculation = default(2); // multiplier for the sigma in range calculation
    double radiusApproximatedMap = default(25);		// radius for 
    double statisticsTime @unit(s) = default (5s);	// time for the vector statistics
    double otherScanToSendProbability = default (0.3);	// [0, 1] probability to send a scanning made by others nodes
    int k_over_n = default(1);			// k over n rule (1=OR, 0=AND, 2..=k-over-n)
    bool radiusApproximatedMapFromDecorrelatedDist = default(false);	// radiusApproximatedMap is calculated as the decorrelation distance
    bool schematicScouting = default(false);	// flag to set a grid schematic exploring technique
    double staticScanCellSize @unit(m) = default(50m); // if "schematicScouting" this parameter define de cell scan size
    bool saveGridMaps = default(true); // flag to save the various maps
    
    @display("i=misc/joystick");
}
