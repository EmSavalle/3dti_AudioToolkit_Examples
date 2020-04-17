
#include <HRTF/HRTFFactory.h>
#include <HRTF/HRTFCereal.h>
#include <BRIR/BRIRFactory.h>
#include <BRIR/BRIRCereal.h>
#include <ILD/ILDCereal.h>
#include <RtAudio.h>
#include <BinauralSpatializer/3DTI_BinauralSpatializer.h>
#include "RtAudio.h"
using namespace Common;
/*
void LoadMyWav(std::vector<float>& samplesVector, const char* stringIn)
{
	struct WavHeader								 // Local declaration of wav header struct type (more info in http://soundfile.sapp.org/doc/WaveFormat/)
	{												 // We only need the number of samples, so the rest will be unused assuming file is mono, 16-bit depth and 44.1kHz sampling rate
		char		  fill[40];
		uint32_t	bytesCount;
	} wavHeader;

	FILE* wavFile = fopen(stringIn, "rb");											 // Opening of the wav file
	fread(&wavHeader, sizeof(wavHeader), 1, wavFile);								 // Reading of the 44 bytes of header to get the number of samples of the file
	fseek(wavFile, sizeof(wavHeader), SEEK_SET);									 // Moving of the file pointer to the start of the audio samples

	unsigned int samplesCount = wavHeader.bytesCount / 2;							 // Getting number of samples by dividing number of bytes by 2 because we are reading 16-bit samples
	int16_t* sample; sample = new int16_t[samplesCount];							 // Declaration and initialization of 16-bit signed integer pointer
	memset(sample, 0, sizeof(int16_t) * samplesCount);								 // Setting its size

	uint8_t* byteSample; byteSample = new uint8_t[2 * samplesCount];				 // Declaration and initialization of 8-bit unsigned integer pointer
	memset(byteSample, 0, sizeof(uint8_t) * 2 * samplesCount);						 // Setting its size

	fread(byteSample, 1, 2 * samplesCount, wavFile);								 // Reading the whole file byte per byte, needed for endian-independent wav parsing

	for (int i = 0; i < samplesCount; i++)
		sample[i] = int16_t(byteSample[2 * i] | byteSample[2 * i + 1] << 8);		 // Conversion from two 8-bit unsigned integer to a 16-bit signed integer

	samplesVector.reserve(samplesCount);											 // Reserving memory for samples vector

	for (int i = 0; i < samplesCount; i++)
		samplesVector.push_back((float)sample[i] / (float)INT16_MAX);				 // Converting samples to float to push them in samples vector
}
int saw(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void* userData)
{
	unsigned int i, j;
	double* buffer = (double*)outputBuffer;
	double* lastValues = (double*)userData;
	if (status)
		std::cout << "Stream underflow detected!" << std::endl;
	// Write interleaved audio data.
	for (i = 0; i < nBufferFrames; i++) {
		for (j = 0; j < 2; j++) {
			*buffer++ = lastValues[j];
			lastValues[j] += 0.005 * (j + 1 + (j * 0.1));
			if (lastValues[j] >= 1.0) lastValues[j] -= 2.0;
		}
	}
	return 0;
}

int maingh() {
	std::cout << "Blah";


	// 1. Create instance(s) of core class (one per listener)
	Binaural::CCore   myCore;

	// 2. Set values of Sample Rate and Buffer Size into the core. By default are set to 44100 and 512.
	int sampleRate = 48000; // Sampling Rate {24000, 44100, 48000, 96000, ...} 
	int bufferSize = 256; 
	myCore.SetAudioState({ sampleRate, bufferSize });
	// 3. Set HRTF resampling step. By default is set to 5.  
	int HRTF_resamplingStep = 45;
	myCore.SetHRTFResamplingStep(HRTF_resamplingStep);

	// LISTENER INITIALIZATION
	// 4. Create instance of listener class.
	shared_ptr<Binaural::CListener> listener = myCore.CreateListener();
	
		// SOURCE CREATION 
		// 5a. Create instance(s) of new source(s) (you can create as many sources as you need)
		shared_ptr<Binaural::CSingleSourceDSP> mySource;
	// 5b. Ask the core to initialize the source (for every sound source)
	mySource = myCore.CreateSingleSourceDSP();

	//SELECT SPATIALISATION MODE
	// 6. Set the spatialization mode, HIGH QUALITY (by default), HIGH PERFORMANCE or NONE. 
	mySource->SetSpatializationMode(Binaural::TSpatializationMode::HighQuality);

	// HIGH QUALITY MODE INTIALIZATION
	// 7a. Load HRTF file, from a SOFA or 3DTI file, into the CHRTF head of the listener.
	string hrtfSofaFile_PATH = "D:/Dev/Audio/3dti_AudioToolkit_Examples/3dti_AudioToolkit/resources/HRTF/SOFA/3DTI_HRTF_IRC1008_128s_44100Hz.sofa";
	bool result = HRTF::CreateFromSofa(hrtfSofaFile_PATH, listener);
	if (result) { cout << "HRTF has been loaded successfully\n"; }


	string fileILDNearFieldEffectPath = "D:/Dev/Audio/3dti_AudioToolkit_Examples/3dti_AudioToolkit/resources/ILD/NearFieldCompensation_ILD_44100.3dti-ild";
	// 7b. Load ILD for Near Field effect from 3DTI file.
	result = ILD::CreateFrom3dti_ILDNearFieldEffectTable(fileILDNearFieldEffectPath, listener);
	if (result) { cout << "ILD Near Field Effect simulation file has been loaded successfully\n"; }
	
	
	//Set position audio	
	std::mutex audioMutex;
	CTransform newSourceTrf;
	
	newSourceTrf.SetPosition(CVector3(5, 5, 5));	//Move source to absolute position
	{
		lock_guard < mutex > lock(audioMutex);
		mySource->SetSourceTransform(newSourceTrf);
	}

	// 1. Declare anechoic output mix, which consists of a pair of mono buffers (one for each channel/ear)  
	Common::CEarPair<CMonoBuffer<float>> bAnechoicOutput;
	bAnechoicOutput.left.resize(bufferSize);
	bAnechoicOutput.right.resize(bufferSize);

	// 3. Get input chunk for this source 
	// Your audio framework should provide you with the necessary methods to obtain the chunk
	CMonoBuffer< float > bInput(bufferSize);

	//bInput.SetFromWhiteNoise();
	LoadMyWav(bInput,"Wasp-sound.wav");
	std::cout << "Wav Loaded\n";
	// 4. Declare output for this source. Core assumes output is a pair of mono buffers (one for each channel/ear)
	Common::CEarPair<CMonoBuffer<float>> singleSourceAnechoicOut;
	std::cout << "singleSourceAnechoic init\n";

	// 5. Spatialise this source, updating the input buffer and passing the output buffer
	lock_guard< mutex > lock(audioMutex);
	mySource->SetBuffer(bInput);
	std::cout << "Source's buffers set\n";
	mySource->ProcessAnechoic(singleSourceAnechoicOut.left, singleSourceAnechoicOut.right);
	std::cout << "Anechoic processed\n";

		// 6. Add this source output to the anechoic output mix
	bAnechoicOutput.left += singleSourceAnechoicOut.left;
	bAnechoicOutput.right += singleSourceAnechoicOut.right;

	// 7. Mix with other non-spatialized sounds, apply other effects,...and finally pass result to audio framework. You may probably need to convert your pair of mono buffers into an array of stereo interlaced float samples. Assuming fOutput is the array of floats (float*) used by your audio framework, you could do:
	int stSample = 0;
	int nb = bAnechoicOutput.left.GetNsamples() * 2;
	
	//allocate the array
	float* arr = new float [nb];
	std::list<float> fOutPut;
	int sample = 0;
	for (int i = 0; i < bAnechoicOutput.left.GetNsamples(); i++)
	{
		arr[sample++] = bAnechoicOutput.left[i];
		arr[sample++] = bAnechoicOutput.right[i];
	}

	std::cout << "b";
	RtAudio dac;
	if (dac.getDeviceCount() < 1) {
		std::cout << "\nNo audio devices found!\n";
		exit(0);
	}
	else {
		std::cout << "\n Audio devices found, Number :" << dac.getDeviceCount();
	}
	RtAudio::StreamParameters parameters;
	parameters.deviceId = dac.getDefaultOutputDevice();
	parameters.nChannels = 2;
	parameters.firstChannel = 0;
	unsigned int bufferFrames = 256; // 256 sample frames
	double data[2];
	try {
		dac.openStream(&parameters, NULL, RTAUDIO_FLOAT64,
			sampleRate, &bufferFrames, &saw, (void*)&bInput);
		dac.startStream();
	}
	catch (RtAudioError& e) {
		e.printMessage();
		exit(0);
	}
	std::cout << "c";

	char input;
	std::cout << "\nPlaying ... press <enter> to quit.\n";
	std::cin.get(input);
	try {
		// Stop the stream
		dac.stopStream();
	}
	catch (RtAudioError& e) {
		e.printMessage();
	}
	if (dac.isStreamOpen()) dac.closeStream();
	std::cout << "d";
	return 2;
}
*/