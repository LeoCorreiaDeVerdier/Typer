/*
 ____  _____ _        _
| __ )| ____| |      / \
|  _ \|  _| | |     / _ \
| |_) | |___| |___ / ___ \
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
	Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
	Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/

#include <Bela.h>
#include <cmath>
#include <Heavy_bela.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <DigitalChannelManager.h>
#include <algorithm>
#include <array>
#include <vector>
//#include <io

#define BELA_HV_SCOPE

#ifdef BELA_HV_DISABLE_SCOPE
#undef BELA_HV_SCOPE
#endif // BELA_HV_DISABLE_SCOPE

/*
 *  MODIFICATION
 *  ------------
 *  Global variables for tremolo effect applied to libpd output
 */

float gTremoloRate = 4.0;
float gPhase;

bool gSensorIn [6][8];
bool gSensorOut [6][8];
int gSensorDeb [6][8];
bool gXSensorIn [8];
bool gXSensorOut [8];
bool gXSensorActive [8] = {true, true, true, true, true, true, true, true};
bool gXSensorInverted [8] = {true, false, false, false, false, false, false, false};
int gXSensorDeb [8];
int gHitDeb = 0;
// float gHitSensor;

int gAudioFramesPerAnalogFrame = 0;
int gHitSensorChannel = 0;

/*********/

enum { minFirstDigitalChannel = 10 };

static unsigned int gAudioChannelsInUse;
static unsigned int gAnalogChannelsInUse;
static unsigned int gDigitalChannelsInUse;
static unsigned int gChannelsInUse;
static unsigned int gFirstAnalogChannel;
static unsigned int gFirstDigitalChannel;
static unsigned int gDigitalChannelOffset;

static unsigned int gDigitalSigInChannelsInUse;
static unsigned int gDigitalSigOutChannelsInUse;

static unsigned int gFirstScopeChannel;
unsigned int gScopeChannelsInUse;
#ifdef BELA_HV_SCOPE
#include <libraries/Scope/Scope.h>
// Bela Scope
float* gScopeOut;
static Scope* scope = NULL;
#endif // BELA_HV_SCOPE
static char multiplexerArray[] = {"bela_multiplexer"};
static int multiplexerArraySize = 0;
static bool pdMultiplexerActive = false;
bool gDigitalEnabled = 0;

/*
 *	HEAVY CONTEXT & BUFFERS
 */

HeavyContextInterface *gHeavyContext;
float *gHvInputBuffers = NULL, *gHvOutputBuffers = NULL;
unsigned int gHvInputChannels = 0, gHvOutputChannels = 0;
uint32_t multiplexerTableHash;

float gInverseSampleRate;

/*
 *	HEAVY FUNCTIONS
 */


void printHook(HeavyContextInterface *context, const char *printLabel, const char *msgString, const HvMessage *msg) {
	const double timestampSecs = ((double) hv_msg_getTimestamp(msg)) / hv_getSampleRate(context);
	rt_printf("print: [@ %.3f] %s: %s\n", timestampSecs, printLabel, msgString);
}


// digitals
static DigitalChannelManager dcm;

void sendDigitalMessage(bool state, unsigned int delay, void* receiverName){
	hv_sendFloatToReceiver(gHeavyContext, hv_stringToHash((char*)receiverName), (float)state);
//	rt_printf("%s: %d\n", (char*)receiverName, state);
}

std::vector<std::string> gHvDigitalInHashes;
void generateDigitalNames(unsigned int numDigitals, unsigned int digitalOffset, std::vector<std::string>& receiverInputNames)
{
	std::string inBaseString = "bela_digitalIn";
	for(unsigned int i = 0; i<numDigitals; i++)
	{
		receiverInputNames.push_back(inBaseString + std::to_string(i+digitalOffset));
	}
}

// utility in case you are having trouble matching types.
void heavyPrintMsgTypes(const HvMessage* m)
{
	for(unsigned int n = 0; n < hv_msg_getNumElements(m); ++n)
	{
		if(hv_msg_isFloat(m, n))
			printf("%c", 'f');
		if(hv_msg_isSymbol(m, n))
			printf("%c", 's');
		if(hv_msg_isBang(m, n))
			printf("%c", 'b');
	}
	printf("\n");
}

// For a message to be received here, you need to use the following syntax in Pd:
// [send receiverName @hv_param]
static void sendHook(
		HeavyContextInterface *context,
		const char *receiverName,
		hv_uint32_t sendHash,
		const HvMessage *m) {
#if 0 // print incoming messages for test purposes. If a message doesn't show up, remember to add @hv_param to the receiver name.
	char* str = hv_msg_toString(m);
	printf("sendHook: %s =>", str);
	heavyPrintMsgTypes(m);
	free(str);
#endif
	/*
	 *  MODIFICATION
	 *  ------------
	 *  Parse float sent to receiver 'tremoloRate' and assign it to a global variable
	 */

	if(strncmp(receiverName, "tremoloRate", 11) == 0){
		float value = hv_msg_getFloat(m, 0); // see the Heavy C API documentation: https://github.com/giuliomoro/hvcc/blob/master-bela/docs/06.cpp.md
		gTremoloRate = value;
	}

	/*********/
	// Bela digital run-time messages

	// TODO: this first block is almost an exact copy of libpd's code, should we add this to the class?
	// let's make this as optimized as possible for built-in digital Out parsing
	// the built-in digital receivers are of the form "bela_digitalOutXX" where XX is between 11 and 26
	static const int prefixLength = 15; // strlen("bela_digitalOut")
	if(strncmp(receiverName, "bela_digitalOut", prefixLength)==0){
		if(receiverName[prefixLength] != 0){ //the two ifs are used instead of if(strlen(source) >= prefixLength+2)
			if(receiverName[prefixLength + 1] != 0){
				// quickly convert the suffix to integer, assuming they are numbers, avoiding to call atoi
				int receiver = ((receiverName[prefixLength] - '0') * 10);
				receiver += (receiverName[prefixLength+1] - '0');
				unsigned int channel = receiver - gDigitalChannelOffset; // go back to the actual Bela digital channel number
				bool value = (hv_msg_getFloat(m, 0) != 0.0f);
				if(channel < gDigitalChannelsInUse){ //gDigitalChannelsInUse is the number of digital channels
					dcm.setValue(channel, value);
				}
			}
		}
		return;
	}

	// More MIDI and digital messages. To obtain the hashes below, use hv_stringToHash("yourString")
	switch (sendHash) {
		case 0x70418732: { // bela_setDigital
			if(gDigitalEnabled)
			{
				// Third argument (optional) can be ~ or sig for signal-rate, message-rate otherwise.
				// [in 14 ~(
				// |
				// [s bela_setDigital]
				// is signal("sig" or "~") or message("message", default) rate
				bool isMessageRate = true; // defaults to message rate
				bool direction = 0; // initialize it just to avoid the compiler's warning
				bool disable = false;
				if (!(hv_msg_isSymbol(m, 0) && hv_msg_isFloat(m, 1))) return;
				const char *symbol = hv_msg_getSymbol(m, 0);
				if(strcmp(symbol, "in") == 0){
					direction = INPUT;
				} else if(strcmp(symbol, "out") == 0){
					direction = OUTPUT;
				} else if(strcmp(symbol, "disable") == 0){
					disable = true;
				} else {
					return;
				}
				int channel = hv_msg_getFloat(m, 1) - gDigitalChannelOffset;
				if(disable == true){
					dcm.unmanage(channel);
					return;
				}
				if(hv_msg_isSymbol(m, 2)){
					const char *s = hv_msg_getSymbol(m, 2);
					if(strcmp(s, "~") == 0  || strncmp(s, "sig", 3) == 0){
						isMessageRate = false;
					}
				}
				dcm.manage(channel, direction, isMessageRate);
			}
			break;
		}
		default: {
			break;
		}
	}
}


/*
 * SETUP, RENDER LOOP & CLEANUP
 */


bool setup(BelaContext *context, void *userData)	{
	/*
	 *  MODIFICATION
 	 *  ------------
	 *  Initialise variables for tremolo effect
	 */

	gPhase = 0.0;

    pinMode(context, 0, 0, OUTPUT); // Set gOutputPin as output
    pinMode(context, 0, 1, OUTPUT); // Set gOutputPin as output
    pinMode(context, 0, 2, OUTPUT); // Set gOutputPin as output
    pinMode(context, 0, 3, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 4, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 5, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 6, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 7, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 8, INPUT); // Set gOutputPin as input
    /*l  pinMode(context, 0, 9, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 10, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 11, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 12, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 13, INPUT); // Set gOutputPin as input
    pinMode(context, 0, 14, INPUT); // Set gOutputPin as input l*/
    pinMode(context, 0, 15, INPUT); // Set gOutputPin as input
    
    gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
    printf("AnalogDensity: %u\n", gAudioFramesPerAnalogFrame);

	/*********/

	// Check if digitals are enabled
	if(context->digitalFrames > 0 && context->digitalChannels > 0)
		gDigitalEnabled = 0; //excdig ( = 1; )
    
	gAudioChannelsInUse = std::max(context->audioInChannels, context->audioOutChannels);
	gAnalogChannelsInUse = std::max(context->analogInChannels, context->analogOutChannels);
	gDigitalChannelsInUse = context->digitalChannels;

	// Channel distribution
	gFirstAnalogChannel = std::max(context->audioInChannels, context->audioOutChannels);
	gFirstDigitalChannel = gFirstAnalogChannel + std::max(context->analogInChannels, context->analogOutChannels);
	if(gFirstDigitalChannel < minFirstDigitalChannel)
		gFirstDigitalChannel = minFirstDigitalChannel; //for backwards compatibility
	gDigitalChannelOffset = gFirstDigitalChannel + 1;
	gFirstScopeChannel = gFirstDigitalChannel + gDigitalChannelsInUse;

	gChannelsInUse = gFirstScopeChannel + gScopeChannelsInUse;

	// Create hashes for digital channels
	generateDigitalNames(gDigitalChannelsInUse, gDigitalChannelOffset, gHvDigitalInHashes);

	/* HEAVY */

	gHeavyContext = hv_bela_new_with_options(context->audioSampleRate, 1000, 200, 0);

	gHvInputChannels = hv_getNumInputChannels(gHeavyContext);
	gHvOutputChannels = hv_getNumOutputChannels(gHeavyContext);

	gScopeChannelsInUse = gHvOutputChannels > gFirstScopeChannel ?
			gHvOutputChannels - gFirstScopeChannel : 0;
	if(gDigitalEnabled)
	{
		gDigitalSigInChannelsInUse = gHvInputChannels > gFirstDigitalChannel ?
			gHvInputChannels - gFirstDigitalChannel : 0;
		gDigitalSigOutChannelsInUse = gHvOutputChannels > gFirstDigitalChannel ?
			gHvOutputChannels - gFirstDigitalChannel - gScopeChannelsInUse: 0;
	}
	else
	{
		gDigitalSigInChannelsInUse = 0;
		gDigitalSigOutChannelsInUse = 0;
	}

	printf("Starting Heavy context with %d input channels and %d output channels\n",
			gHvInputChannels, gHvOutputChannels);
	printf("Channels in use:\n");
	printf("Digital in : %u, Digital out: %u\n", gDigitalSigInChannelsInUse, gDigitalSigOutChannelsInUse);
#ifdef BELA_HV_SCOPE
	printf("Scope out: %u\n", gScopeChannelsInUse);
#endif // BELA_HV_SCOPE

	if(gHvInputChannels != 0) {
		gHvInputBuffers = (float *)calloc(gHvInputChannels * context->audioFrames,sizeof(float));
	}
	if(gHvOutputChannels != 0) {
		gHvOutputBuffers = (float *)calloc(gHvOutputChannels * context->audioFrames,sizeof(float));
	}

	gInverseSampleRate = 1.0 / context->audioSampleRate;

	// Set heavy print hook
	hv_setPrintHook(gHeavyContext, printHook);
	// Set heavy send hook
	hv_setSendHook(gHeavyContext, sendHook);

	if(gScopeChannelsInUse > 0){
#if __clang_major__ == 3 && __clang_minor__ == 8
		fprintf(stderr, "Scope currently not supported when compiling heavy with clang3.8, see #265 https://github.com/BelaPlatform/Bela/issues/265. You should specify `COMPILER gcc;` in your Makefile options\n");
		exit(1);
#endif
#ifdef BELA_HV_SCOPE
		scope = new Scope();
		scope->setup(gScopeChannelsInUse, context->audioSampleRate);
		gScopeOut = new float[gScopeChannelsInUse];
#endif // BELA_HV_SCOPE
	}
	// Bela digital
	if(gDigitalEnabled)
	{
		dcm.setCallback(sendDigitalMessage);
		if(gDigitalChannelsInUse> 0){
			for(unsigned int ch = 0; ch < gDigitalChannelsInUse; ++ch){
				dcm.setCallbackArgument(ch, (void *) gHvDigitalInHashes[ch].c_str());
			}
		}
	}
	// unlike libpd, no need here to bind the bela_digitalOut.. receivers
	// but make sure you do something like [send receiverName @hv_param]
	// when you want to send a message from Heavy to the wrapper.
	multiplexerTableHash = hv_stringToHash(multiplexerArray);
	if(context->multiplexerChannels > 0){
		pdMultiplexerActive = true;
		multiplexerArraySize = context->multiplexerChannels * context->analogInChannels;
		hv_table_setLength(gHeavyContext, multiplexerTableHash, multiplexerArraySize);
		hv_sendFloatToReceiver(gHeavyContext, hv_stringToHash("bela_multiplexerChannels"), context->multiplexerChannels);
	}
//    hv_sendMessageToReceiverV(gHeavyContext, hv_stringToHash("sendfromhvcc"), 0.0f, "s", "success");
	return true;
}

void render(BelaContext *context, void *userData)
{
	// De-interleave the data
	if(gHvInputBuffers != NULL) {
		for(unsigned int n = 0; n < context->audioFrames; n++) {
			for(unsigned int ch = 0; ch < gHvInputChannels; ch++) {
				if(ch >= gAudioChannelsInUse + gAnalogChannelsInUse) {
					// THESE ARE PARAMETER INPUT 'CHANNELS' USED FOR ROUTING
					// 'sensor' outputs from routing channels of dac~ are passed through here
					// these could be also digital channels (handled by the dcm)
					// or parameter channels used for routing (currently unhandled)
					break;
				} else {
					// If more than 2 ADC inputs are used in the pd patch, route the analog inputs
					// i.e. ADC3->analogIn0 etc. (first two are always audio inputs)
					if(ch >= gAudioChannelsInUse)
					{
						unsigned int analogCh = ch - gAudioChannelsInUse;
						if(analogCh < context->analogInChannels)
						{
							int m = n/2;
							float mIn = analogRead(context, m, analogCh);
							gHvInputBuffers[ch * context->audioFrames + n] = mIn;
						}
					} else {
						if(ch < context->audioInChannels)
							gHvInputBuffers[ch * context->audioFrames + n] = audioRead(context, n, ch);
					}
				}
			}
		}
	}

	if(pdMultiplexerActive){
		static int lastMuxerUpdate = 0;
		if(++lastMuxerUpdate == multiplexerArraySize){
			lastMuxerUpdate = 0;
			memcpy(hv_table_getBuffer(gHeavyContext, multiplexerTableHash), (float *const)context->multiplexerAnalogIn, multiplexerArraySize * sizeof(float));
		}
	}


	// Bela digital in
	if(gDigitalEnabled)
	{
		// note: in multiple places below we assume that the number of digital frames is same as number of audio
		// Bela digital in at message-rate
		dcm.processInput(context->digital, context->digitalFrames);
	}

	// replacement for bang~ object
	//hv_sendMessageToReceiverV(gHeavyContext, "bela_bang", 0.0f, "b");

	// heavy audio callback
	hv_processInline(gHeavyContext, gHvInputBuffers, gHvOutputBuffers, context->audioFrames);
	/*
	for(int n = 0; n < context->audioFrames*gHvOutputChannels; ++n)
	{
		printf("%.3f, ", gHvOutputBuffers[n]);
		if(n % context->audioFrames == context->audioFrames - 1)
			printf("\n");
	}
	*/

	// Bela digital out
	if(gDigitalEnabled)
	{
		// Bela digital out at signal-rate
		if(gDigitalSigOutChannelsInUse > 0)
		{
				unsigned int j, k;
				float *p0, *p1;
				const unsigned int gLibpdBlockSize = context->audioFrames;
				const unsigned int audioFrameBase = 0;
				float* gOutBuf = gHvOutputBuffers;
				// block below copy/pasted from libpd, except
				// context->digitalChannels has been replaced with gDigitalSigOutChannelsInUse
				for (j = 0, p0 = gOutBuf; j < gLibpdBlockSize; ++j, ++p0) {
					unsigned int digitalFrame = (audioFrameBase + j);
					for (k = 0, p1 = p0 + gLibpdBlockSize * gFirstDigitalChannel;
							k < gDigitalSigOutChannelsInUse; k++, p1 += gLibpdBlockSize) {
						if(dcm.isSignalRate(k) && dcm.isOutput(k)){ // only process output channels that are handled at signal rate
							digitalWriteOnce(context, digitalFrame, k, *p1 > 0.5);
						}
					}
				}
		}
		// Bela digital out at message-rate
		dcm.processOutput(context->digital, context->digitalFrames);
	}

#ifdef BELA_HV_SCOPE
	// Bela scope
	if(gScopeChannelsInUse > 0)
	{
		unsigned int j, k;
		float *p0, *p1;
		const unsigned int gLibpdBlockSize = context->audioFrames;
		float* gOutBuf = gHvOutputBuffers;

		// block below copy/pasted from libpd
		for (j = 0, p0 = gOutBuf; j < gLibpdBlockSize; ++j, ++p0) {
			for (k = 0, p1 = p0 + gLibpdBlockSize * gFirstScopeChannel; k < gScopeChannelsInUse; k++, p1 += gLibpdBlockSize) {
				gScopeOut[k] = *p1;
			}
			scope->log(gScopeOut);
		}
	}
#endif // BELA_HV_SCOPE

	// Interleave the output data
	if(gHvOutputBuffers != NULL) {
		/*
		 *  MODIFICATION
		 *  ------------
		 *  Processing for tremolo effect while writing heavy output to Bela output buffer
		 */

		// Generate a sinewave with frequency set by gTremoloRate
		// and amplitude from -0.5 to 0.5
		float lfo = sinf(gPhase) * 0.5;
		// Keep track and wrap the phase of the sinewave
		gPhase += 2.0 * M_PI * gTremoloRate * gInverseSampleRate;
		if(gPhase > 2.0 * M_PI)
			gPhase -= 2.0 * M_PI;
        
        
        for(unsigned int n = 0; n < 8; n++) {
            for(unsigned int m = 0; m < 6; m++){
                if(m == 5 && n > 3) {
                    gSensorIn[m][n] = 0;
                } else {
                    gSensorIn[m][n] = !digitalRead(context, n, m + 3);
                }
                if(gSensorIn[m][n]) {
                    gSensorDeb[m][n] = 256;
                }
                if(gSensorIn[m][n] != gSensorOut[m][n]) {
                    if(gSensorIn[m][n]) {
                        gSensorOut[m][n] = 1;
                        hv_sendMessageToReceiverV(gHeavyContext, hv_stringToHash("keystatus"), 0.0f, "ff", (float) m * 8 + n, 1.0f);
                    } else {
                        if(gSensorDeb[m][n] == 0) {
                            gSensorOut[m][n] = 0;
                            hv_sendMessageToReceiverV(gHeavyContext, hv_stringToHash("keystatus"), 0.0f, "ff", (float) m * 8 + n, 0.0f);
                        } else {
                            gSensorDeb[m][n]--;
                        }
                    }
                }
            }
            if(gXSensorActive[n]) {
                if(gXSensorInverted[n]){
                    gXSensorIn[n] = digitalRead(context, n, 15);
                } else {
                    gXSensorIn[n] = !digitalRead(context, n, 15);
                }
            } else {
                gXSensorIn[n] = false;
            }
            if(gXSensorIn[n] != gXSensorOut[n]) {
                if(gXSensorIn[n]) {
                    gXSensorOut[n] = 1;
                    hv_sendMessageToReceiverV(gHeavyContext, hv_stringToHash("xkeystatus"), 0.0f, "ff", (float) n, 1.0f);
                } else {
                    if(gXSensorDeb[n] == 0) {
                        gXSensorOut[n] = 0;
                        hv_sendMessageToReceiverV(gHeavyContext, hv_stringToHash("xkeystatus"), 0.0f, "ff", (float) n, 0.0f);
                    } else {
                        gXSensorDeb[n]--;
                    }
                }
            }
        }
        
        for(unsigned int n = 0; n < context->audioFrames; n++) {
            if(gAudioFramesPerAnalogFrame && !(n % gAudioFramesPerAnalogFrame)) {
                float hitSensor = analogRead(context, n/gAudioFramesPerAnalogFrame, gHitSensorChannel);
                if(hitSensor < 0.1) {
                    if(gHitDeb == 0) {
                        hv_sendBangToReceiver(gHeavyContext, hv_stringToHash("nedslag1"));
                    }
                    gHitDeb = 1024;
                } else {
                    if(hitSensor > 0.4 && gHitDeb > 0) {
                        gHitDeb--;
                    }
                }
                    
            }
        }

		/*********/
		for(unsigned int n = 0; n < context->audioFrames; n++) {
			for(unsigned int ch = 0; ch < gHvOutputChannels; ch++) {
				if(ch >= gAudioChannelsInUse + gAnalogChannelsInUse) {
					// THESE ARE SENSOR OUTPUT 'CHANNELS' USED FOR ROUTING
					// they are the content of the 'sensor output' dac~ channels
				} else {
					if(ch >= gAudioChannelsInUse)	{
						int m = n/2;
						unsigned int analogCh = ch - gAudioChannelsInUse;
						if(analogCh < context->analogOutChannels)
							analogWriteOnce(context, m, analogCh, gHvOutputBuffers[ch*context->audioFrames + n]);
					} else {
						if(ch < context->audioOutChannels)
							audioWrite(context, n, ch, gHvOutputBuffers[ch * context->audioFrames + n] * lfo); // MODIFICATION (* lfo)
					}
				}
			}
		}
        // Write the adresser to digital pins 0-2
        for(unsigned int n = 0; n < context->digitalFrames; ++n){
            digitalWrite(context, n, 0, (bool)(n & 4));
            digitalWrite(context, n, 1, (bool)(n & 2));
            digitalWrite(context, n, 2, (bool)(n & 1));
        }
        //
	}
    
//    hv_sendMessageToReceiverV(gHeavyContext, hv_stringToHash("sendfromhvcc"), 0.0f, "s", "success");
}


void cleanup(BelaContext *context, void *userData)
{
	hv_delete(gHeavyContext);
	free(gHvInputBuffers);
	free(gHvOutputBuffers);
#ifdef BELA_HV_SCOPE
	delete[] gScopeOut;
	delete scope;
#endif // BELA_HV_SCOPE
}
