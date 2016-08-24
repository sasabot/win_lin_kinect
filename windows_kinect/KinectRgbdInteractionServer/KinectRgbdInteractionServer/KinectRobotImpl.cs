using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grpc.Core;
using Grpc.Core.Utils;
using System.Speech.Synthesis;
using System.Threading;

namespace Kinectrobot
{
    class KinectRobotImpl : KinectRobot.KinectRobotBase
    {
        // rgbd streaming

        private ReaderWriterLockSlim pointsLocker = null;

        private Kinectrobot.Points[] pointBlobs = null;

        // speech

        private SpeechSynthesizer speechSynthesizer;

        private ReaderWriterLockSlim locker = null;

        private bool abortSpeech = false;

        public bool triggerAfterRecognition = false;

        private bool triggerDuringSpeech = false;

        // logs

        public delegate void LogInfo(string blockName, string text, bool concatenate = false);

        private LogInfo logInfo = null;

        public KinectRobotImpl(LogInfo logFunction, int divideStream)
        {
            // create lockers
            this.pointsLocker = new ReaderWriterLockSlim();
            this.locker = new ReaderWriterLockSlim();

            // setup for logs
            this.logInfo = new LogInfo(logFunction);

            // setup for rgbd streaming
            this.pointBlobs = new Kinectrobot.Points[divideStream];
            for (int i = 0; i < this.pointBlobs.Length; ++i)
            {
                this.pointBlobs[i] = new Kinectrobot.Points { };
            }

            // setup for TTS
            this.speechSynthesizer = new SpeechSynthesizer();
            this.speechSynthesizer.SelectVoiceByHints(VoiceGender.Female, VoiceAge.Teen);
            this.speechSynthesizer.SetOutputToDefaultAudioDevice();
            this.speechSynthesizer.SpeakCompleted += this.SpeechEnd_FrameArrived;
        }

        public void Dispose()
        {
            if (this.speechSynthesizer != null)
            {
                this.speechSynthesizer.Dispose();
            }
        }

        public void SetPoints(Kinectrobot.Points points)
        {
            int pointsPerStream = points.Data.Count / this.pointBlobs.Length + 1;

            this.pointsLocker.EnterWriteLock();
            try
            {
                for (int i = 0; i < this.pointBlobs.Length; ++i)
                {
                    this.pointBlobs[i].Data.Clear();
                    int range = System.Math.Min((i + 1) * pointsPerStream, points.Data.Count);
                    for (int j = pointsPerStream * i; j < range; ++j)
                        this.pointBlobs[i].Data.Add(points.Data[j]);
                }
            }
            finally { this.pointsLocker.ExitWriteLock(); }
        }

        public override async Task ReturnPoints(Request request, IServerStreamWriter<Points> responseStream, ServerCallContext context)
        {
            this.pointsLocker.EnterReadLock();
            try
            {
                foreach (var blob in this.pointBlobs)
                {
                    await responseStream.WriteAsync(blob);
                }
            }
            finally { this.pointsLocker.ExitReadLock(); }
        }

        public bool RecognitionOffTriggerOn()
        {
            bool abortFlag = false;

            locker.EnterReadLock();
            try { abortFlag = this.abortSpeech; }
            finally { locker.ExitReadLock(); }

            return abortFlag;
        }

        public void RecognitionOff()
        {
            locker.EnterWriteLock();
            try { this.abortSpeech = true; }
            finally { locker.ExitWriteLock(); }
        }

        public void RecognitionOn()
        {
            locker.EnterWriteLock();
            try { this.abortSpeech = false; }
            finally { locker.ExitWriteLock(); }
        }

        public override Task<Response> SendSpeech(Speech request, ServerCallContext context)
        {
            if (request.Command == "quit")
            {
                this.speechSynthesizer.SpeakAsyncCancelAll();
                return Task.FromResult(ReturnTrue());
            }

            this.speechSynthesizer.SpeakAsyncCancelAll();

            if (this.triggerDuringSpeech)
            {
                locker.EnterWriteLock();
                try { this.abortSpeech = true; }
                finally { locker.ExitWriteLock(); }
            }

            // example of speech
            // raw: "U--m... yeah?"
            // ssml: "<prosody rate=\"x-slow\">Um</prosody><break time=\"0.5s\"/><break time=\"0.5s\"/> yeah?"
            string raw_string = request.Speech_;
            // convert "--" syntax to x-slow
            while (true)
            {
                int getDashFromHead = raw_string.IndexOf("--");
                if (getDashFromHead < 0) break;
                // index should not change as erase and addition will happen on index behind
                int wordStart = raw_string.LastIndexOf(" ", getDashFromHead);
                int ssmlCheck = raw_string.LastIndexOf(">", getDashFromHead); // space could be inside ssml format
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf(".", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf(",", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf("?", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = raw_string.LastIndexOf("!", getDashFromHead);
                if (wordStart < ssmlCheck || wordStart < 0) wordStart = 0; // no spaces prior should mean head of sentence
                else wordStart += 1;
                // insert end first
                int wordEnd = raw_string.IndexOf(" ", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf(".", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf(",", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf("?", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.IndexOf("!", getDashFromHead);
                if (wordEnd < 0) wordEnd = raw_string.Length - 1;
                raw_string = raw_string.Insert(wordEnd, "</prosody>");
                // erase dash dash
                raw_string = raw_string.Remove(getDashFromHead, 2);
                raw_string = raw_string.Insert(wordStart, "<prosody rate=\"x-slow\">");
            }
            // convert "..." syntax to break
            raw_string.Replace("...", "<break time=\"0.5s\">");
            Prompt speech = new Prompt("<?xml version=\"1.0\"?><speak version=\"1.0\" xml:lang=\"en-US\">" + raw_string + "</speak>", SynthesisTextFormat.Ssml);
            this.speechSynthesizer.SpeakAsync(speech);

            return Task.FromResult(ReturnTrue());
        }

        public void SpeechEnd_FrameArrived(object sender, SpeakCompletedEventArgs e)
        {
            if (this.triggerDuringSpeech)
            {
                locker.EnterWriteLock();
                try { this.abortSpeech = false; }
                finally { locker.ExitWriteLock(); }
            }
        }

        public override Task<Response> SetSTTBehavior(VoiceTriggers request, ServerCallContext context)
        {
            if (request.ManualTriggerOn)
            {
                this.abortSpeech = true;
                return Task.FromResult(ReturnTrue());
            }

            if (request.ManualTriggerOff)
            {
                this.abortSpeech = false;
                return Task.FromResult(ReturnTrue());
            }

            this.triggerAfterRecognition = request.AutoTriggerAfterRecognition;

            this.triggerDuringSpeech = request.AutoTriggerDuringSpeech;

            return Task.FromResult(ReturnTrue());
        }

        public override Task<Response> WebAgent(UrlInfo request, ServerCallContext context)
        {
            // if no search within webpage
            if (request.Key == "")
            {
                System.Diagnostics.Process.Start(request.Url);
                return Task.FromResult(ReturnTrue());
            }

            System.Net.WebClient wc = new System.Net.WebClient();
            string raw = wc.DownloadString(request.Url);
            List<string> substrings = new List<string> { };
            int getString = 0;
            string headString = request.Linkhead;
            string keyString = request.Key;
            while ((getString = raw.IndexOf(keyString, getString)) >= 0)
            {
                int findHref = raw.LastIndexOf("href=", getString) + 6;
                if (findHref < 0) continue;
                substrings.Add(headString + raw.Substring(findHref, getString - findHref) + keyString);
                getString += keyString.Length;
            }

            int order = -1;
            if (request.Style == "random") // return a random match
            {
                Random rand = new Random();
                System.Diagnostics.Process.Start(substrings[rand.Next(0, substrings.Count - 1)]);
            }
            else if (int.TryParse(request.Style, out order)) // return ith match
            {
                if (order < substrings.Count && order >= 0)
                    System.Diagnostics.Process.Start(substrings[order]);
            }
            
            wc.Dispose();

            return Task.FromResult(ReturnTrue());
        }

        private Response ReturnTrue()
        {
            return new Response { Status = true };
        }
    }
}
