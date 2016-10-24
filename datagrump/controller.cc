#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define SCHEME(3)
#define INITIAL_CWND (15.0)
#define AI (1.0)
#define MD (0.5)
#define DELAY_THRES (100)
#define EWMA_ALPHA (0.5)
#define TIMEOUT (100)
#define TICK_LEN (20.0)
#define NUM_RATIOS ((int) 50)
#define TARGET_DELAY (100)
#define DELAY_TICKS ((int) (TARGET_DELAY / TICK_LEN))
#define MIN_CWND (10.0)
#define PERC (0.10)


/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
  previous_ack_(0),
  cwnd_(INITIAL_CWND),
  scheme_(SCHEME),
  rtt_(0.0),
  last_tick_(-20),
  lambda_(1.0),
  ratio_(0.5),
  prev_firsts_(),
  prev_lambdas_(),
  cursor_(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  unsigned int the_window_size = (unsigned int) cwnd_;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
     << " window size is " << the_window_size << endl;
  }

  return the_window_size;
}

int compareDoubles (const void * a, const void * b) {
  if (*(double*)a > *(double*)b)
    return 1;
  else if (*(double*)a < *(double*)b)
    return -1;
  else
    return 0;
}

/* updates prediction of ratio between future cumulative lambdas and current lambda_ */
void Controller::update_ratio( void )
{
  double ratios [NUM_RATIOS] = { };
  for ( int i = 0; i < NUM_RATIOS; i++ ) {a
    /* computes the NUM_RATIOS previously observed ratios */
    ratios[i] = ((prev_firsts_[i+DELAY_TICKS] - prev_firsts_[i])/(DELAY_TICKS * TICK_LEN)) / prev_lambdas_[i];
  }
  qsort(ratios,NUM_RATIOS,sizeof(double),compareDoubles);
  /* take predition of ratio to be PERC percentile among observed ratios */
  ratio_ = ratios[(int) (PERC*NUM_RATIOS)];
  for ( int i = 0; i < DELAY_TICKS; i++ ) {
    prev_firsts_[i] = prev_firsts_[i + NUM_RATIOS];
    prev_lambdas_[i] = prev_lambdas_[i + NUM_RATIOS];
  }
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
                    /* of the sent datagram */
                    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
     << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
                   /* what sequence number was acknowledged */
                   const uint64_t send_timestamp_acked,
                   /* when the acknowledged datagram was sent (sender's clock) */
                   const uint64_t recv_timestamp_acked,
                   /* when the acknowledged datagram was received (receiver's clock)*/
                   const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
     << " received ack for datagram " << sequence_number_acked
     << " (send @ time " << send_timestamp_acked
     << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
     << endl;
  }

  /* AIMD, Warmup B */
  if ( scheme_ == 1 ) {
    int ack_delay = timestamp_ack_received - previous_ack_;
    if ( ack_delay > (int) timeout_ms() ) {
    //cerr << "ACK_DELAY" << ack_delay << endl;
      /* timeout happened - multiplicative decrease*/
      cwnd_ *= MD;
    } else {
      /* no congestion signal - additive increase */
      cwnd_ +=  AI / cwnd_;
    }
    cwnd_= max(cwnd_, 1.0);
    previous_ack_ = timestamp_ack_received;
  }

  /* Delay-triggered, Warmup C */
  if ( scheme_ == 2 ){
    uint64_t rtt_now = timestamp_ack_received - send_timestamp_acked;
    /* compute new rtt as EWMA */
    rtt_ = (unsigned int) rtt_now * EWMA_ALPHA + rtt_ * (1 - EWMA_ALPHA);
    if ( rtt_ > DELAY_THRES ) {
      /* Delay too long - multiplicative decrease */
      cwnd_ *= MD;
    } else {
      /* Delay just fine - additive increase */
      cwnd_ += AI / cwnd_;
    }
    cwnd_ = max(cwnd_, 1.0);
  }

  /* Fanciness, Exercise D */
  if ( scheme_ == 3 ){
    int tick_gap = (int) (timestamp_ack_received - last_tick_);
    
    if ( tick_gap >= 20 ) {
      /* we've entered a new tick
      update all prev_firsts_ that haven't been updated, leading up to current tick
      to be equal to the sequence number of the packet just acked */
      for ( int i = 1; i <= tick_gap/20; i++ ) {
        prev_firsts_[cursor_] = sequence_number_acked;
        if ( cursor_ > 0 ) {
          uint64_t last_tick_packets = prev_firsts_[cursor_] - prev_firsts_[cursor_-1];
          double lambda_now = last_tick_packets/TICK_LEN;
          lambda_ = lambda_now * EWMA_ALPHA + lambda_ * (1 - EWMA_ALPHA);
          prev_lambdas_[cursor_] = lambda_;
        }
        cursor_ = (cursor_ + 1);
        
        /* once we've collected enough ratio observations, update ratio
        prediction and cycle cursor back */
        if ( cursor_ == NUM_RATIOS + DELAY_TICKS  ) {
          update_ratio();
          cursor_ = DELAY_TICKS;
        }
      }
      /* compute beginning time of the current tick and update last_tick_ accordingly */
      last_tick_ = (timestamp_ack_received/TICK_LEN)*TICK_LEN;
      /* compute new window size based on new value of lambda and prediction of ratio */
      cwnd_ = max(MIN_CWND,lambda_ * ratio_ * TARGET_DELAY);
    }
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return TIMEOUT; /* timeout of one second */
}