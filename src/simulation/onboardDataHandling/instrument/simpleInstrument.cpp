/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "simpleInstrument.h"
#include "../../simMessages/dataNodeUsageSimMsg.h"

/*! Constructor, which sets the default nodeDataOut to zero.
 @return void
*/
SimpleInstrument::SimpleInstrument(){
    this->nodeBaudRate = 0.0;
    return;
}

/*! Destructor.
 @return void
 */
SimpleInstrument::~SimpleInstrument(){
    return;
}

/*! Sets the name and baud rate for the data in the output message.
 @return void
*/
void SimpleInstrument::evaluateDataModel(DataNodeUsageSimMsg *dataUsageSimMsg, double currentTime){
    dataUsageSimMsg->baudRate = this->nodeBaudRate;
    strncpy (dataUsageSimMsg->dataName, this->nodeDataName, sizeof(dataUsageSimMsg->dataName));
    return;
}