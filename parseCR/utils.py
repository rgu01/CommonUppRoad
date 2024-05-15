def write_large_block(file):
    large_block = """
typedef int[-1,65535] id_t;

typedef struct {
    int32_t x;
    int32_t y;
}ST_DPOINT;

typedef struct {
    double x;
    double y;
}ST_CPOINT;

typedef struct {
    ST_DPOINT ends[2];
}ST_DLINE;

typedef struct {
    ST_DPOINT points[MAXP];
    bool dashLine;
}ST_BOUND;

typedef struct {
    id_t ID;
    ST_BOUND left;
    ST_BOUND right;
    id_t predecessor[MAXPRE];
    id_t successor[MAXSUC];
    id_t adjLeft;
    bool dirLeft;
    id_t adjRight;
    bool dirRight;
}ST_LANE;

typedef struct {
    bool collide; // collision
    bool outside; // outside road
    bool reach;   // reach the goal
}ST_DETECTION;

typedef struct {
    ST_CPOINT position;
    double velocity;
    double orientation;
    double acceleration;
    double yawRate;
}ST_CSTATE;

typedef struct {
    ST_DPOINT position;
    int16_t velocity;
    int16_t orientation;
    int16_t acceleration;
    int16_t yawRate;
    ST_DETECTION detection;
}ST_DSTATE;

typedef struct {
    hybrid clock x;
    hybrid clock y;
    hybrid clock velocity;
    hybrid clock orientation;
    hybrid clock acceleration;
    hybrid clock yawRate;
}ST_VARIABLES;

typedef struct {
    ST_DPOINT center;
    int16_t width;
    int16_t length;
    int16_t orientation;
}ST_RECTANGLE;

typedef struct {
    int maxVelocity;
    int minVelocity;
    int maxOrientation;
    int minOrientation;
}ST_RULES;

typedef struct {
    ST_DPOINT goal;
}ST_PLANNING;

typedef struct {
    int32_t time;
    ST_CSTATE cState;
}ST_PAIR;
"""
    file.write(large_block)