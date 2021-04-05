package me.drton.jmavsim;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;

public class PeripherialBuzzer {
    BlockingQueue<Note> notes = new LinkedBlockingQueue<>();
    AudioFormat af;
    SourceDataLine sdl;
    float sample_rate = 44100; // assume a sample rate of 44.1 kHz

    class Note {
        public int frequency;
        public int duration;
        public Note(int frequency, int duration) {
            this.frequency = Integer.valueOf(frequency);
            this.duration = duration;
        }
    }

    public void playNote(int note, int duration) {
        notes.add(new Note(note, duration));
    }

    public class NoteConsumer implements Runnable {
        public void run() {
            while (true) {
                try {
                    Note timedNote = notes.take();
                    generateTone(timedNote.frequency, timedNote.duration);
                } catch (LineUnavailableException e) {
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public PeripherialBuzzer() {
        Thread myThread = new Thread(new NoteConsumer());
        af = new AudioFormat(sample_rate, 8, 1, true, false);
        try {
            sdl = AudioSystem.getSourceDataLine(af);
            sdl.open(af);
        } catch (LineUnavailableException e) {
            e.printStackTrace();
        }

        sdl.start();

        myThread.start();
    }

    public void generateTone(int hz, int msecs) throws LineUnavailableException {
        byte[] buf = new byte[2048];

        int k = 0;

        for (int i = 0; i < msecs * sample_rate / 1000;) {
            // fill up the local buffer
            for (k = 0; k < 2047 && (i < msecs * sample_rate / 1000); i++, k++) {
                buf[k] = (byte)(Math.sin(i / (sample_rate / hz) * 2.0 * Math.PI) * 100);
            }

            sdl.write(buf, 0, k);
        }

        sdl.drain();
    }
}
