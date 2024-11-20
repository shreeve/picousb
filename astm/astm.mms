flowchart

Idle                  --> |Receive ENQ| Awake
Awake                 --> |Busy, Send NAK| Idle
Awake                 --> |Send ACK, Seq = 1, Timer = 30| Waiting_Recv
Waiting_Recv          --> |Receive EOT or Timeout| Idle
Waiting_Recv          --> |Get Frame| Frame_Received
Frame_Received        --> |Frame OK| Have_Data
Frame_Received        --> |Good Frame, Increment Seq, Send ACK, Timer = 30| Waiting_Recv
Frame_Received        --> |Bad Frame, Send NAK, Timer = 30| Waiting_Recv
Have_Data             --> |New Frame, Increment Seq, Send EOT, Timer = 30| Waiting_Recv

Idle                  --> |Data available| Data_to_Send
Data_to_Send          --> |Seq = 1, Send ENQ, Timer = 15| Waiting_Send
Data_to_Send          --> |Busy| Contention_Busy_Timer
Contention_Busy_Timer --> Idle
Waiting_Send          --> |Timeout, Send EOT| Idle
Waiting_Send          --> |Receive ENQ or NAK| Contention_Busy_Timer
Waiting_Send          --> |Receive ACK, Retries = 0| Next_Frame
Next_Frame            --> Frame_Ready
Next_Frame            --> |Done, Send EOT| Idle
Frame_Ready           --> |Send Frame, Timer = 15| Waiting_After
Waiting_After         --> |Receive NAK, Increment Retries; Or, any other character| Old_Frame
Waiting_After         --> |Increment Seq, Receive ACK, Retries = 0| Next_Frame
Waiting_After         --> |Timeout, Send EOT| Idle
Waiting_After         --> |Receive EOT| Interrupt_Requested
Old_Frame             --> |Retries = 6, Send EOT| Idle
Old_Frame             --> |Retries < 6| Frame_Ready
Interrupt_Requested   --> |Ignore, Retries = 0, Increment Seq| Next_Frame
Interrupt_Requested   --> |Accept, Send EOT| Idle
