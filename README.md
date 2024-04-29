
`FinalProjectPublisher.py` and `FinalProjectSubscriber.py` utilizes ZMQ to mitigate the issue of the microphone's insensitivity to the buzzer sound by sending a message to stop the predator car to listen for the sound.

The code in `MP-V3` does not use ZMQ. The predator car constantly listens for and turns successfully towards sounds like voices and claps but not the buzzer.

Disregard `ProjectThread.py`.
