///
/// \file 			TestUtil.hpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2017-11-24
/// \last-modified 	2017-11-24
/// \brief			Contains utility methods to help with testing.
/// \details
///					See README.rst in repo root dir for more info.

#ifndef MN_CPP_LINUX_SERIAL_TEST_UTIL_H_
#define MN_CPP_LINUX_SERIAL_TEST_UTIL_H_

// System includes
#include <string>
#include <array>
#include <memory>
#include <iostream>
#include <thread>
#include <chrono>

// 3rd party includes


using namespace std::literals;

#define READ   0
#define WRITE  1
FILE * popen2(std::string command, std::string type, int & pid)
{
    pid_t child_pid;
    int fd[2];
    pipe(fd);

    if((child_pid = fork()) == -1)
    {
        perror("fork");
        exit(1);
    }

    /* child process */
    if (child_pid == 0)
    {
        if (type == "r")
        {
            close(fd[READ]);    //Close the READ end of the pipe since the child's fd is write-only
            dup2(fd[WRITE], 1); //Redirect stdout to pipe
        }
        else
        {
            close(fd[WRITE]);    //Close the WRITE end of the pipe since the child's fd is read-only
            dup2(fd[READ], 0);   //Redirect stdin to pipe
        }

        setpgid(child_pid, child_pid); //Needed so negative PIDs can kill children of /bin/sh
        execl("/bin/sh", "/bin/sh", "-c", command.c_str(), NULL);
        exit(0);
    }
    else
    {
        if (type == "r")
        {
            close(fd[WRITE]); //Close the WRITE end of the pipe since parent's fd is read-only
        }
        else
        {
            close(fd[READ]); //Close the READ end of the pipe since parent's fd is write-only
        }
    }

    pid = child_pid;

    if (type == "r")
    {
        return fdopen(fd[READ], "r");
    }

    return fdopen(fd[WRITE], "w");
}

int pclose2(FILE * fp, pid_t pid)
{
    int stat;

    fclose(fp);
    while (waitpid(pid, &stat, 0) == -1)
    {
        if (errno != EINTR)
        {
            stat = -1;
            break;
        }
    }

    return stat;
}

struct ProcessInfo {
    FILE* fp;
    pid_t pid;
};


namespace mn {
    namespace CppLinuxSerial {

        class TestUtil {

        public:
            /// \brief      Executes a command on the Linux command-line.
            /// \details    Blocks until command is complete.
            /// \throws     std::runtime_error is popen() fails.
            static std::string Exec(const std::string &cmd) {
                std::array<char, 128> buffer;
                std::string result;
                std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
                if (!pipe) throw std::runtime_error("popen() failed!");

                while (!feof(pipe.get())) {
                    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
                        result += buffer.data();
                }

                return result;
            }

            void StartProcess(const std::string &cmd) {
                std::array<char, 128> buffer;
                std::string result;
                int pid;
                FILE * fp = popen2(cmd, "r", pid);
                ProcessInfo processInfo;
                processInfo.fp = fp;
                processInfo.pid = pid;
                processes_.push_back(processInfo);
            }

            void CreateVirtualSerialPortPair() {
                std::cout << "Creating virtual serial port pair..." << std::endl;
//                StartProcess("sudo socat -d -d pty,raw,echo=0,link=/dev/ttyS10 pty,raw,echo=0,link=/dev/ttyS11");
//                std::this_thread::sleep_for(1s);
//                StartProcess("sudo chmod a+rw /dev/ttyS10");
//                StartProcess("sudo chmod a+rw /dev/ttyS11");
//                std::this_thread::sleep_for(1s);
//                std::cout << "Finished creating virtual serial port pair." << std::endl;
//                std::system("./run.sh");
                std::system("nohup sudo socat -d -d pty,raw,echo=0,link=/dev/ttyS10 pty,raw,echo=0,link=/dev/ttyS11 &");
                std::this_thread::sleep_for(1s);
                std::system("sudo chmod a+rw /dev/ttyS10");
                std::system("sudo chmod a+rw /dev/ttyS11");
            }

            void CloseSerialPorts() {
//                for(const auto& filePointer : processes_) {
//                    std::cout << "Sending SIGINT..." << std::endl;
//                    kill(filePointer.pid, SIGINT);
//                    std::cout << "Calling pclose2()..." << std::endl;
//                    pclose2(filePointer.fp, filePointer.pid);
//                }
                std::system("sudo pkill socat");
            }

            std::vector<ProcessInfo> processes_;

        };
    } // namespace CppLinuxSerial
} // namespace mn

#endif // #ifndef MN_CPP_LINUX_SERIAL_TEST_UTIL_H_
