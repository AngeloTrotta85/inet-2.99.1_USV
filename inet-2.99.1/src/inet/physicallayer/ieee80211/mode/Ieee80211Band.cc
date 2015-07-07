//
// Copyright (C) 2014 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "inet/physicallayer/ieee80211/mode/Ieee80211Band.h"

namespace inet {

namespace physicallayer {

Ieee80211BandBase::Ieee80211BandBase(const char *name) :
    name(name)
{
}

Ieee80211EnumeratedBand::Ieee80211EnumeratedBand(const char *name, const std::vector<Hz> centers) :
    Ieee80211BandBase(name),
    centers(centers)
{
}

Hz Ieee80211EnumeratedBand::getCenterFreqency(int channelNumber) const
{
    if (channelNumber < 0 || channelNumber >= (int)centers.size())
        throw cRuntimeError("Invalid channel number: %d", channelNumber);
    return centers[channelNumber];
}

Ieee80211ArithmeticalBand::Ieee80211ArithmeticalBand(const char *name, Hz start, Hz spacing, int numChannels) :
    Ieee80211BandBase(name),
    start(start),
    spacing(spacing),
    numChannels(numChannels)
{
}

Hz Ieee80211ArithmeticalBand::getCenterFreqency(int channelNumber) const
{
    if (channelNumber < 0 || channelNumber >= numChannels)
        throw cRuntimeError("Invalid channel number: %d", channelNumber);
    return start + spacing * channelNumber;
}

const Ieee80211EnumeratedBand Ieee80211CompliantBands::bandTVWS("TVWS",
{
        MHz(474),   // 0
        MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),      // from 1 to 10  //to define
        MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),MHz(474),      // from 11 to 20 //to define
        MHz(474),   // 21
        MHz(482),   // 22
        MHz(490),   // 23
        MHz(498),   // 24
        MHz(506),   // 25
        MHz(514),   // 26
        MHz(522),   // 27
        MHz(530),   // 28
        MHz(538),   // 29
        MHz(546),   // 30
        MHz(554),   // 31
        MHz(562),   // 32
        MHz(570),   // 33
        MHz(578),   // 34
        MHz(586),   // 35
        MHz(594),   // 36
        MHz(602),   // 37
        MHz(610),   // 38
        MHz(618),   // 39
        MHz(626),   // 40
        MHz(634),   // 41
        MHz(642),   // 42
        MHz(650),   // 43
        MHz(658),   // 44
        MHz(666),   // 45
        MHz(674),   // 46
        MHz(682),   // 47
        MHz(690),   // 48
        MHz(698),   // 49
        MHz(706),   // 50
        MHz(714),   // 51
        MHz(722),   // 52
        MHz(730),   // 53
        MHz(738),   // 54
        MHz(746),   // 55
        MHz(754),   // 56
        MHz(762),   // 57
        MHz(770),   // 58
        MHz(778),   // 59
        MHz(786),   // 60
});

const Ieee80211EnumeratedBand Ieee80211CompliantBands::band2_4GHz("2.4 GHz",
{
    GHz(2.412),    // 1
    GHz(2.417),    // 2
    GHz(2.422),    // 3
    GHz(2.427),    // 4
    GHz(2.432),    // 5
    GHz(2.437),    // 6
    GHz(2.442),    // 7
    GHz(2.447),    // 8
    GHz(2.452),    // 9
    GHz(2.457),    // 10
    GHz(2.462),    // 11
    GHz(2.467),    // 12
    GHz(2.472),    // 13
    GHz(2.484),    // 14, this channel is intentionally further away from the previous than the others, see 802.11 specification
});

const Ieee80211ArithmeticalBand Ieee80211CompliantBands::band5GHz("5 GHz", GHz(5), MHz(5), 200);

const Ieee80211ArithmeticalBand Ieee80211CompliantBands::band5_9GHz("5.9 GHz", GHz(5.86), MHz(10), 7);

const std::vector<const IIeee80211Band *> Ieee80211CompliantBands::bands = {&bandTVWS, &band2_4GHz, &band5GHz, &band5_9GHz};

const IIeee80211Band *Ieee80211CompliantBands::findBand(const char *name)
{
    for (auto & band : bands)
        if (!strcmp(band->getName(), name))
            return band;
    return nullptr;
}

const IIeee80211Band *Ieee80211CompliantBands::getBand(const char *name)
{
    const IIeee80211Band *band = findBand(name);
    if (band == nullptr)
        throw cRuntimeError("Unknown 802.11 band: '%c'", name);
    else
        return band;
}

} // namespace physicallayer

} // namespace inet

