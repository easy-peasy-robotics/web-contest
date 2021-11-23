/*
 * Copyright (C) 2020 iCub Tech Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/all.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;

/**********************************************************************/
class TestAssignmentWebContestSandbox : public yarp::robottestingframework::TestCase
{
    RpcClient servicePort;
    RpcClient worldPort;

public:
    /******************************************************************/
    TestAssignmentWebContestSandbox() :
        yarp::robottestingframework::TestCase("TestAssignmentWebContestSandbox") {}

    /******************************************************************/
    virtual ~TestAssignmentWebContestSandbox() {}

    /******************************************************************/
    bool setup(Property& property) override
    {
        float rpcTmo=(float)property.check("rpc-timeout",Value(240.0)).asFloat64();

        worldPort.open("/"+getName()+"/world:rpc");
        worldPort.asPort().setTimeout(rpcTmo);
        if (!Network::connect(worldPort.getName(), "/world/rpc")) {
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /world/rpc");
        }

        servicePort.open("/"+getName()+"/service:rpc");
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Set rpc timeout = %g [s]",rpcTmo));
        servicePort.asPort().setTimeout(rpcTmo);
        if (!Network::connect(servicePort.getName(), "/service")) {
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /service");
        }

        return true;
    }

    /******************************************************************/
    void tearDown() override
    {
        worldPort.close();
        servicePort.close();
    }

    /******************************************************************/
    void run() override
    {
        unsigned int score{0};
        for (int attempt = 1; attempt <= 3; attempt++) {
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Starting attempt #%d...", attempt));
            int response_correct, response_actual;
            
            {
                Bottle cmd, rep;
                cmd.addVocab32("shuffle");
                worldPort.write(cmd, rep);
                response_correct = rep.get(1).asInt32();
                Time::delay(5.);
            }

            {
                Bottle cmd, rep;
                cmd.addVocab32("go");
                if (servicePort.write(cmd, rep)) {
                    response_actual = rep.get(0).asInt32();
                } else {
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /service");
                }
            }

            if (response_actual == response_correct) {
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Attempt #%d result: got it!", attempt));
                score += 1;
            } else {
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Attempt #%d result: damn'!", attempt));
            }
        }

        ROBOTTESTINGFRAMEWORK_TEST_CHECK(score > 1, Asserter::format("Total score = %d", score));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentWebContestSandbox)
