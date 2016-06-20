// host name, port and send object
"localhost" => string hostname;
7000 => int port;
OscSend xmitPeriod;
xmitPeriod.setHost( hostname, port );

7001 => port;
OscSend xmitBT;
xmitBT.setHost( hostname, port );

// variables to check if music is off
1 => int noMusic;
0 => float intensity;
0.04 => float offThreshold;
0 => int offCount;
150 => int countThreshold;

// prepare the short-time fft
512 => int fftsize;
adc => FFT fft => blackhole;
fftsize*2 => fft.size;
Windowing.hann(256) => fft.window;
polar prevSpec[fftsize];
float diffSingleFreq;
float sumDiffs;

// normalisation variables
0.05 => float adaptability;
0 => float instantMean;
1000 => float instantStd;
float normDiff;

// infinite time loop
while(true)
{
    // Get sound intensity and FFT and sum up the difference with previous one in each frequency
    fft.upchuck();
    for (0 => int i; i < fftsize; i++) {
        (fft.cval(i)$polar).mag - prevSpec[i].mag => diffSingleFreq;
        (fft.cval(i)$polar).mag + intensity => intensity;
        if (diffSingleFreq > 0) {
            diffSingleFreq + sumDiffs => sumDiffs;
        }
    }
    
    // normalise signal
    (1 - adaptability)*instantMean + adaptability*sumDiffs => instantMean;
    (1 - adaptability)*instantStd + adaptability*Std.fabs(sumDiffs-instantMean) => instantStd;
    if(instantStd < 0.01) {
        0.01 => instantStd;
        //<<< "low instantStd:", instantStd >>>;
        //<<< "mean:", instantMean >>>;
    }
    (sumDiffs - instantMean)/instantStd => normDiff;
    
	//<<< intensity >>>;
    // send the results of the analysis if there is music
    checkMusicIsOff(intensity);
    if(!noMusic) {
        xmitPeriod.startMsg( "/onset", "f" );
        xmitBT.startMsg( "/onset", "f" );
        normDiff => xmitPeriod.addFloat;
        normDiff => xmitBT.addFloat;
        if(sumDiffs > 0.03) {
            <<< "sent (via OSC):", normDiff >>>;
        }
    } else {
        xmitPeriod.startMsg( "/onset", "f" );
        -1000 => xmitPeriod.addFloat;
        xmitBT.startMsg( "/onset", "f" );
        -1000 => xmitBT.addFloat;
    }
    
    // update previous spectrum and advance time
    0 => intensity;
    0 => sumDiffs;
    for(0 => int i; i < fftsize; i++) {
        fft.cval(i)$polar => prevSpec[i];
    }
    4::ms => now;
}

// Functions and utilities

fun void checkMusicIsOff(float soundInt) {
    if(soundInt < offThreshold) {
        if(offCount > countThreshold) {
            1 => noMusic;
            <<< "Music OFF!" >>>;
            return;
        }
        offCount++;
    } else {
        offCount - 6 => offCount;
        if(offCount < 0) {
            0 => offCount;
            0 => noMusic;
            <<< "Music ON!" >>>;
        }
    }
}