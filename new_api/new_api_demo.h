

long unsigned count_xenomai_mode_switches = 0;

void action_upon_switch(int sig __attribute__((unused)));

std::vector<CAN_CanConnection_t> can_connections;

void cleanup_and_exit(int sig);


class CanFrame
{

 public:

    std::array<uint8_t, 8> data;
    uint8_t dlc;
    can_id_t id;
    nanosecs_abs_t timestamp;
    // \todo: do we need this?
    int recv_ifindex;

};


class CanConnection
{

 public:

    struct sockaddr_can send_addr;
    int socket;

};


#define FLOAT_TO_Q24(fval) ((int)(fval * (1 << 24)))


class CanBus

{

 public:

    typedef std::function<void(CanFrame)> Callback;

 private:

    CanConnection can_connection_;
    RT_MUTEX can_connection_mutex_;

    std::vector<Callback> callbacks_;
    unsigned callback_count_;
    RT_MUTEX callback_mutex_;

    RT_TASK rt_task_;
    bool rt_task_is_running_;
    RT_MUTEX rt_task_mutex_;

 public: 

    CanBus(std::string can_interface_name, unsigned max_callback_count = 10);
    void add_callback(Callback callback);
    void loop();
    static void close_can(int socket);
    static void loop(void* instance_pointer);
    void start_loop();
    int join();
    void send_frame(uint32_t id, uint8_t *data, uint8_t dlc);

 private:

    static CAN_CanConnection_t setup_can(std::string can_interface_name);
    CanFrame receive_frame();

};


class Board
{

 // \todo: add time stamps!
 private:

    // should probably make this a shared pointer
    std::shared_ptr<CanBus> can_bus_;

    BLMC_BoardData_t data_;
    RT_MUTEX data_mutex_;

    // controls
    Eigen::Vector2d current_targets_;
    RT_MUTEX current_targets_mutex_;

 public:

    Board(std::shared_ptr<CanBus> can_bus);
    void enable();
    void consume_can_frame(CanFrame frame);
    void send_command(uint32_t cmd_id, int32_t value);
    void set_current_targets(Eigen::Vector2d currents);
    void set_current_target(double current, unsigned motor_id);
    double get_current_measurement(unsigned motor_id);
    double get_position_measurement(unsigned motor_id);
    double get_velocity_measurement(unsigned motor_id);
    double get_encoder_measurement(unsigned motor_id);
    double get_analog_measurement(unsigned adc_id);

private:

    unsigned id_to_index(unsigned motor_id);
};



class Motor
{

    // \todo: should probably make this a shared pointer
    std::shared_ptr<Board> board_;
    unsigned motor_id_;

 public:

    Motor(std::shared_ptr<Board> board, unsigned motor_id);
    double get_current_measurement();
    double get_position_measurement();
    double get_velocity_measurement();
    double get_encoder_measurement();
    void set_current_target(double current_target);

};


class AnalogSensor
{

    std::shared_ptr<Board> board_;
    unsigned sensor_id_;

 public:

    AnalogSensor(std::shared_ptr<Board> board, unsigned sensor_id);
    double get_analog_measurement();

};


class Controller
{

 private:

    RT_TASK rt_task_;
    // \todo: should probably make this a shared pointer
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<AnalogSensor> analog_sensor_;

 public:

    Controller(std::shared_ptr<Motor> motor, std::shared_ptr<AnalogSensor> analog_sensor);
    void start_loop();
    static void loop(void* instance_pointer);
    void loop();
    int join();

};


