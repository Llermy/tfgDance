#ifndef osc_float_receiver_h
#define osc_float_receiver_h

#define ONSET_SIGNAL_PERIOD_SEC 0.016

class OscFloatReceiver
{
    
private:
    int listener_d;
    
    int open_listener();
    void bind_to_port(int socket, int port);
    float parse_float32(char *float_string);

public:
    OscFloatReceiver(int port);
    float read_float();
};


#endif /* osc_float_receiver_h */
