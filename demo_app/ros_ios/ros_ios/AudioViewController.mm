//
//  AudioViewController.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-04-11.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "AudioViewController.h"
#import <vector>

@interface AudioViewController ()

@end

#define kOutputBus 0
#define kInputBus 1

static OSStatus recordingCallback(void * inRefCon,
                                  AudioUnitRenderActionFlags * ioActionFlags,
                                  const AudioTimeStamp * inTimeStamp,
                                  UInt32 inBusNumber,
                                  UInt32 inNumberFrames,
                                  AudioBufferList * ioData)
{
    AudioViewController * iosAudio = (__bridge AudioViewController *)inRefCon;
    
    // Because of the way our audio format (setup below) is chosen:
    // we only need 1 buffer, since it is mono
    // Samples are 16 bits = 2 bytes.
    // 1 frame includes only 1 sample
    
    AudioBufferList bufferList;
    bufferList.mNumberBuffers = 1;
    bufferList.mBuffers[0].mData = NULL;
    
    // Obtain recorded samples
    OSStatus result;
    
    result = AudioUnitRender([iosAudio audioUnit],
                             ioActionFlags,
                             inTimeStamp,
                             inBusNumber,
                             inNumberFrames,
                             &bufferList);
    
    // Now, we have the samples we just read sitting in buffers in bufferList
    // Process the new data
    [iosAudio processAudio:&bufferList];
    
    return result;
}

static OSStatus playbackCallback(void *inRefCon,
                                 AudioUnitRenderActionFlags *ioActionFlags,
                                 const AudioTimeStamp *inTimeStamp,
                                 UInt32 inBusNumber,
                                 UInt32 inNumberFrames,
                                 AudioBufferList *ioData)
{
    AudioViewController * iosAudio = (__bridge AudioViewController *)inRefCon;
    
    // Notes: ioData contains buffers (may be more than one!)
    // Fill them up as much as you can. Remember to set the size value in each buffer to match how
    // much data is in the buffer.
    
    for (int i = 0; i < ioData->mNumberBuffers; i++)
    {
        // in practice we will only ever have 1 buffer, since audio format is mono
        AudioBuffer buffer = ioData->mBuffers[i];
        
        // NSLog(@"  Buffer %d has %d channels and wants %d bytes of data.", i, buffer.mNumberChannels, buffer.mDataByteSize);
        
        iosAudio->ros_controller_->lockAudioBuffer();
        
        std::vector<unsigned char> * input_buffer = iosAudio->ros_controller_->getAudioBuffer();
        UInt8 * data = (UInt8 *)buffer.mData;
        NSUInteger count = input_buffer->size();
        
        //NSLog(@"%d %d", count, (unsigned int)buffer.mDataByteSize);
        
        if(count)
        {
            UInt32 size = MIN(buffer.mDataByteSize, count);
            
            memcpy(data, input_buffer->data(), size);
            
            input_buffer->erase(input_buffer->begin(), input_buffer->begin() + size);
            
            buffer.mDataByteSize = size;
        }
        else
        {
            *ioActionFlags = kAudioUnitRenderAction_OutputIsSilence;
        }
        
        iosAudio->ros_controller_->unlockAudioBuffer();
    }
    
    return noErr;
}

@implementation AudioViewController

@synthesize isPaused, audioUnit, pause;

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    NSLog(@"AudioViewController : viewDidLoad");
    
    ros_controller_ = new RosAudio();
    
    if([self initAudio])
    {
        isPaused = NO;
        [self start];
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"AudioViewController : viewWillAppear");
    ros_controller_->view_controller_ = self;
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"AudioViewController : viewWillDisappear");
    [self stop];
}

-(void)dealloc
{
    NSLog(@"AudioViewController : dealloc");
    OSStatus result = AudioUnitUninitialize(audioUnit);
    if(result == noErr)
        NSLog(@"AudioUnitUninitialize");
    delete ros_controller_;
}

- (BOOL)initAudio
{
    // Describe audio component
    AudioComponentDescription desc;
    desc.componentType = kAudioUnitType_Output;
    desc.componentSubType = kAudioUnitSubType_RemoteIO;
    desc.componentFlags = 0;
    desc.componentFlagsMask = 0;
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;
    
    // Get component
    AudioComponent inputComponent = AudioComponentFindNext(NULL, &desc);
    
    // Get audio units
    OSStatus result = AudioComponentInstanceNew(inputComponent, &audioUnit);
    
    // Enable IO for recording
    UInt32 flag = 1;
    result = AudioUnitSetProperty(audioUnit,
                                  kAudioOutputUnitProperty_EnableIO,
                                  kAudioUnitScope_Input,
                                  kInputBus,
                                  &flag,
                                  sizeof(flag));
    
    // Enable IO for playback
    result =  AudioUnitSetProperty(audioUnit,
                                   kAudioOutputUnitProperty_EnableIO,
                                   kAudioUnitScope_Output,
                                   kOutputBus,
                                   &flag,
                                   sizeof(flag));
    
    // Describe format
    AudioStreamBasicDescription audioFormat;
    audioFormat.mSampleRate = RosAudio::SAMPLE_RATE;
    
    audioFormat.mFormatID = kAudioFormatLinearPCM;
    
    //*
    audioFormat.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked | kAudioFormatFlagIsNonInterleaved;
    audioFormat.mFramesPerPacket = 1;
    audioFormat.mChannelsPerFrame = RosAudio::CHANNELS_NUMBER;
    audioFormat.mBitsPerChannel = 16;
    audioFormat.mBytesPerPacket = 2;
    audioFormat.mBytesPerFrame = 2;
    audioFormat.mReserved = 0;
    //*/
    
    // NSLog(@"%d", IsAudioFormatNativeEndian(audioFormat));
    
    // Apply format
    result = AudioUnitSetProperty(audioUnit,
                                  kAudioUnitProperty_StreamFormat,
                                  kAudioUnitScope_Output,
                                  kInputBus,
                                  &audioFormat,
                                  sizeof(audioFormat));
    
    result = AudioUnitSetProperty(audioUnit,
                                  kAudioUnitProperty_StreamFormat,
                                  kAudioUnitScope_Input,
                                  kOutputBus,
                                  &audioFormat,
                                  sizeof(audioFormat));
    
    AURenderCallbackStruct callbackStruct;
    
    // Set input callback
    callbackStruct.inputProc = recordingCallback;
    callbackStruct.inputProcRefCon = (__bridge void *)(self);
    result = AudioUnitSetProperty(audioUnit,
                                  kAudioOutputUnitProperty_SetInputCallback,
                                  kAudioUnitScope_Global,
                                  kInputBus,
                                  &callbackStruct,
                                  sizeof(callbackStruct));
    
    // Set output callback
    callbackStruct.inputProc = playbackCallback;
    callbackStruct.inputProcRefCon = (__bridge void *)(self);
    result = AudioUnitSetProperty(audioUnit,
                                  kAudioUnitProperty_SetRenderCallback,
                                  kAudioUnitScope_Global,
                                  kOutputBus,
                                  &callbackStruct,
                                  sizeof(callbackStruct));
    
    // Initialise
    result = AudioUnitInitialize(audioUnit);
    
    if(result == noErr)
        return YES;
    else
        return NO;
}

- (void)processAudio: (AudioBufferList*) bufferList
{
    AudioBuffer sourceBuffer = bufferList->mBuffers[0];
    
    std::vector<UInt8> outputBuffer;
    
    outputBuffer.insert(outputBuffer.begin(), (UInt8 *)sourceBuffer.mData, (UInt8 *)sourceBuffer.mData + sourceBuffer.mDataByteSize);
    
    ros_controller_->sendAudio(outputBuffer);
}

- (IBAction)pauseOrResume:(id)sender
{
    if(isPaused)
    {
        pause.title = @"Pause";
    }
    else
    {
        [self stop];
    }
}

- (void)start
{
    OSStatus result = AudioOutputUnitStart(audioUnit);
    if(result != noErr)
    {
        NSLog(@"AudioOutputUnitStart Error !");
        exit(1);
    }
    [self start];
    isPaused = NO;
}

- (void)stop
{
    OSStatus result = AudioOutputUnitStop(audioUnit);
    if(result != noErr)
    {
        NSLog(@"AudioOutputUnitStop Error !");
        exit(1);
    }
    pause.title = @"Resume";
    isPaused = YES;
}
@end
