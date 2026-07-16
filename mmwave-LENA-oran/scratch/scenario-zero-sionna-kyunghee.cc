/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Minimal EN-DC (1 LTE anchor + N mmWave gNB + M UE) scenario with the channel
 * driven by Sionna RT ray tracing against the real Kyunghee campus scene,
 * instead of the statistical ThreeGpp models scenario-zero uses.
 *
 * This is intentionally the smallest scratch that exercises
 * MmWaveHelper::EnableSionna() end to end (positions -> Sionna server ->
 * ray-traced CFR/pathloss/delay back into ns-3's spectrum channel). E2/KPM/RC
 * and energy logging from scenario-zero-with_parallel_loging.cc are added in
 * a later step once this is verified.
 *
 * Companion Python server: contrib/sionna/model/ns3sionna/kyunghee_server.py
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/sionna-mobility-model.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE("ScenarioZeroSionnaKyunghee");

// Kyunghee 캠퍼스 씬 좌표계 (sionna-oai-flexric-smo/sionna_rt/cir_generator.py의
// CAMPUS_CENTER와 맞춤). gNB는 옥상 높이, UE는 보행자 높이.
static const double GNB_Z = 25.0;
static const double UE_Z = 1.5;

struct UeTraceSample
{
    double time{0.0};
    Vector position;
    bool active{true};
};

struct UeTrace
{
    std::string externalId;
    std::vector<UeTraceSample> samples;
};

struct GnbPosition
{
    std::string externalId;
    Vector position;
};

static std::string
Trim(std::string value)
{
    const auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

static bool
ParseActive(const std::string& text)
{
    std::string value = Trim(text);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return std::tolower(ch);
    });
    if (value == "1" || value == "true" || value == "yes" || value == "active")
    {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "inactive")
    {
        return false;
    }
    throw std::runtime_error("invalid active value: " + text);
}

static std::vector<UeTrace>
LoadUeTrace(const std::string& path, uint32_t maxUes)
{
    std::ifstream input(path);
    NS_ABORT_MSG_IF(!input, "Cannot open SUMO UE trace: " << path);

    std::vector<UeTrace> traces;
    std::map<std::string, uint32_t> indexById;
    std::string line;
    uint32_t lineNumber = 0;
    while (std::getline(input, line))
    {
        ++lineNumber;
        line = Trim(line);
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::vector<std::string> fields;
        std::stringstream row(line);
        std::string field;
        while (std::getline(row, field, ','))
        {
            fields.push_back(Trim(field));
        }
        if (!fields.empty() && (fields[0] == "time" || fields[0] == "time_s"))
        {
            continue;
        }
        NS_ABORT_MSG_IF(fields.size() != 6,
                        "SUMO trace line " << lineNumber
                                           << " must be time_s,ue_id,x,y,z,active");

        try
        {
            const std::string& externalId = fields[1];
            auto [it, inserted] = indexById.emplace(externalId, traces.size());
            if (inserted)
            {
                NS_ABORT_MSG_IF(traces.size() >= maxUes,
                                "SUMO trace has more UE IDs than N_Ues=" << maxUes);
                traces.push_back(UeTrace{externalId, {}});
            }
            UeTraceSample sample;
            sample.time = std::stod(fields[0]);
            sample.position =
                Vector(std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4]));
            sample.active = ParseActive(fields[5]);
            traces[it->second].samples.push_back(sample);
        }
        catch (const std::exception& error)
        {
            NS_ABORT_MSG("Invalid SUMO trace line " << lineNumber << ": " << error.what());
        }
    }

    NS_ABORT_MSG_IF(traces.empty(), "SUMO trace contains no UE samples: " << path);
    for (auto& trace : traces)
    {
        std::stable_sort(trace.samples.begin(), trace.samples.end(),
                         [](const auto& a, const auto& b) { return a.time < b.time; });
    }
    return traces;
}

static std::vector<GnbPosition>
LoadGnbPositions(const std::string& path)
{
    std::ifstream input(path);
    NS_ABORT_MSG_IF(!input, "Cannot open gNB position CSV: " << path);

    std::vector<GnbPosition> positions;
    std::string line;
    uint32_t lineNumber = 0;
    while (std::getline(input, line))
    {
        ++lineNumber;
        line = Trim(line);
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::vector<std::string> fields;
        std::stringstream row(line);
        std::string field;
        while (std::getline(row, field, ','))
        {
            fields.push_back(Trim(field));
        }
        if (!fields.empty() && (fields[0] == "gnb_id" || fields[0] == "id"))
        {
            continue;
        }
        NS_ABORT_MSG_IF(fields.size() != 4,
                        "gNB position line " << lineNumber << " must be gnb_id,x,y,z");
        try
        {
            positions.push_back(
                {fields[0], Vector(std::stod(fields[1]), std::stod(fields[2]),
                                   std::stod(fields[3]))});
        }
        catch (const std::exception& error)
        {
            NS_ABORT_MSG("Invalid gNB position line " << lineNumber << ": " << error.what());
        }
    }
    NS_ABORT_MSG_IF(positions.empty(), "gNB position CSV contains no positions: " << path);
    return positions;
}

static bool
InitialActive(const UeTrace& trace)
{
    bool active = false;
    for (const auto& sample : trace.samples)
    {
        if (sample.time > 0.0)
        {
            break;
        }
        active = sample.active;
    }
    return active;
}

static Vector
InitialPosition(const UeTrace& trace)
{
    Vector position = trace.samples.front().position;
    for (const auto& sample : trace.samples)
    {
        if (sample.time > 0.0)
        {
            break;
        }
        position = sample.position;
    }
    return position;
}

int
main(int argc, char* argv[])
{
    uint32_t nMmWaveEnbNodes = 1;
    uint32_t nUes = 1;
    double centerFrequency = 3.5e9;
    double bandwidth = 20e6;
    std::string sionnaServerIp = "tcp://localhost:5555";
    std::string sionnaScene = "Kyunghee.xml";
    bool sionnaVerbose = false;
    double simTime = 10.0;
    bool enableE2 = false;
    std::string e2TermIp = "127.0.0.1";
    bool enableSionna = true;
    double trafficStart = 0.5;
    bool requireTraffic = true;
    bool realtime = false;
    std::string sumoTrace;
    std::string gnbPositions;

    CommandLine cmd(__FILE__);
    cmd.AddValue("N_MmWaveEnbNodes", "Number of mmWave gNBs", nMmWaveEnbNodes);
    cmd.AddValue("N_Ues", "Number of UEs", nUes);
    cmd.AddValue("CenterFrequency", "Center frequency in Hz", centerFrequency);
    cmd.AddValue("Bandwidth", "Bandwidth in Hz", bandwidth);
    cmd.AddValue("sionnaServerIp", "Sionna server ZMQ address", sionnaServerIp);
    cmd.AddValue("sionnaScene", "Scene XML filename (relative to server's --model_folder)",
                 sionnaScene);
    cmd.AddValue("sionnaVerbose", "Enable ns-3-side Sionna logging", sionnaVerbose);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("enableE2", "Report DU/CU-UP/CU-CP KPM to FlexRIC at e2TermIp", enableE2);
    cmd.AddValue("e2TermIp", "FlexRIC nearRT-RIC E2 termination IP", e2TermIp);
    cmd.AddValue("enableSionna", "If false, use the default ThreeGpp channel (diagnostic)",
                 enableSionna);
    cmd.AddValue("trafficStart", "Downlink UDP traffic start time in seconds", trafficStart);
    cmd.AddValue("requireTraffic",
                 "Return a non-zero status if no downlink bytes reach any UE", requireTraffic);
    cmd.AddValue("realtime",
                 "Pace ns-3 virtual time to wall-clock time for external RIC/xApp tests",
                 realtime);
    cmd.AddValue("sumoTrace",
                 "CSV trace: time_s,ue_id,x,y,z,active; enables external UE mobility",
                 sumoTrace);
    cmd.AddValue("gnbPositions", "CSV positions: gnb_id,x,y,z for mmWave gNBs",
                 gnbPositions);
    cmd.Parse(argc, argv);

    const std::vector<UeTrace> ueTraces =
        sumoTrace.empty() ? std::vector<UeTrace>() : LoadUeTrace(sumoTrace, nUes);
    NS_ABORT_MSG_IF(!sumoTrace.empty() && ueTraces.size() != nUes,
                    "N_Ues=" << nUes << " but SUMO trace contains " << ueTraces.size()
                              << " distinct UE IDs; set N_Ues to the exact trace population");
    const std::vector<GnbPosition> configuredGnbPositions =
        gnbPositions.empty() ? std::vector<GnbPosition>() : LoadGnbPositions(gnbPositions);
    NS_ABORT_MSG_IF(!gnbPositions.empty() && configuredGnbPositions.size() != nMmWaveEnbNodes,
                    "N_MmWaveEnbNodes=" << nMmWaveEnbNodes << " but gNB position CSV has "
                                         << configuredGnbPositions.size() << " entries");

    NS_ABORT_MSG_IF(simTime <= 0.0, "simTime must be greater than zero");
    NS_ABORT_MSG_IF(trafficStart < 0.0 || trafficStart >= simTime,
                    "trafficStart must be non-negative and smaller than simTime");

    if (realtime)
    {
        GlobalValue::Bind("SimulatorImplementationType",
                          StringValue("ns3::RealtimeSimulatorImpl"));
    }

    if (sionnaVerbose)
    {
        LogComponentEnable("ScenarioZeroSionnaKyunghee", LOG_LEVEL_INFO);
    }

    // MmWaveEnbNetDevice/LteEnbNetDevice unconditionally try to register with an
    // E2Termination at device-construction time, regardless of whether reporting is
    // actually wanted -- this fork doesn't tolerate these attributes being left unset
    // (crashes with a null Ptr<> dereference). enableE2=false just makes the reports
    // no-ops; it does not skip the registration/construction path itself.
    Config::SetDefault("ns3::MmWaveHelper::E2TermIp", StringValue(e2TermIp));
    Config::SetDefault("ns3::MmWaveHelper::E2ModeLte", BooleanValue(enableE2));
    Config::SetDefault("ns3::MmWaveHelper::E2ModeNr", BooleanValue(enableE2));
    Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue(enableE2));
    Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue(enableE2));
    Config::SetDefault("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue(enableE2));
    Config::SetDefault("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue(enableE2));
    Config::SetDefault("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue(enableE2));
    Config::SetDefault("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue(0.1));
    Config::SetDefault("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue(0.1));
    Config::SetDefault("ns3::LteEnbNetDevice::KPM_E2functionID", DoubleValue(2));
    Config::SetDefault("ns3::MmWaveEnbNetDevice::KPM_E2functionID", DoubleValue(2));
    Config::SetDefault("ns3::LteEnbNetDevice::RC_E2functionID", DoubleValue(3));

    Config::SetDefault("ns3::MmWaveHelper::HarqEnabled", BooleanValue(true));
    Config::SetDefault("ns3::MmWaveHelper::UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue(true));
    Config::SetDefault("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue(100));
    Config::SetDefault("ns3::PhasedArrayModel::AntennaElement",
                       PointerValue(CreateObject<IsotropicAntennaModel>()));

    Config::SetDefault("ns3::McUeNetDevice::AntennaNum", UintegerValue(1));
    Config::SetDefault("ns3::MmWaveNetDevice::AntennaNum", UintegerValue(1));
    Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(centerFrequency));

    // ns-3's mmWave PSD is per-RB, not per-OFDM-subcarrier (see
    // MmWaveSpectrumValueHelper::GetSpectrumModel -- one SpectrumModel band per RB), and
    // SionnaHelper::Configure() silently multiplies both channel_bw and fft_size by
    // GUARD_MULTIPLIER=3 for guard-band margin. SionnaSpectrumPropagationLossModel then asserts
    // the CFR array (fft_size*3 values + 1 trailing guard) has EXACTLY as many entries as the
    // PSD has RBs, or it aborts. So: probe MmWavePhyMacCommon for the actual RB count at the
    // requested bandwidth, and nudge the bandwidth up until (numRb - 1) is a multiple of 3.
    uint32_t numRb = 0;
    double rbWidth = 0.0;
    for (int attempt = 0; attempt < 16; ++attempt)
    {
        Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(bandwidth));
        Ptr<MmWavePhyMacCommon> probeConfig = CreateObject<MmWavePhyMacCommon>();
        numRb = probeConfig->GetNumRb();
        rbWidth = probeConfig->GetRbWidth();
        if (numRb >= 4 && (numRb - 1) % 3 == 0)
        {
            NS_LOG_UNCOND("Bandwidth=" << bandwidth / 1e6 << "MHz -> " << numRb
                                        << " RBs (fits Sionna CFR sizing after "
                                        << attempt << " nudge(s))");
            break;
        }
        bandwidth += 1e6; // widen by 1MHz and retry
    }
    NS_ABORT_MSG_IF((numRb - 1) % 3 != 0,
                    "Could not find a Bandwidth where (numRb-1) is a multiple of 3");
    int sionnaFftSize = static_cast<int>((numRb - 1) / 3);

    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();

    if (enableSionna)
    {
        // Sionna RT를 켠다 -- 반드시 InstallXxxDevice() 이전에 호출해야 함
        // (채널은 DoInitialize()에서 lazy하게 한 번만 만들어짐).
        mmwaveHelper->EnableSionna(sionnaScene, sionnaServerIp);

        // 기본 빔포밍(MmWaveSvdBeamforming)은 ThreeGppChannelModel의 채널 행렬(SVD 입력)이
        // 반드시 있어야 하는데 Sionna 경로엔 그게 없다 -- DFT 빔포밍은 상대 위치 기반
        // steering vector만 쓰므로(채널 행렬 불필요) Sionna와 호환된다.
        mmwaveHelper->SetBeamformingModelType("ns3::MmWaveDftBeamforming");
    }
    else
    {
        // 진단용: Sionna 없이 기본 통계 채널로 같은 토폴로지를 돌려서, MAC/PHY 문제가
        // Sionna 때문인지 이 fork에 원래 있던 것인지 구분한다.
        mmwaveHelper->SetPathlossModelType("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
        mmwaveHelper->SetChannelConditionModelType("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");
        mmwaveHelper->SetChannelModelType("ns3::ThreeGppSpectrumPropagationLossModel");
    }

    Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper>();
    mmwaveHelper->SetEpcHelper(epcHelper);

    // EPC + remote host
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // 노드: LTE 앵커 1 + mmWave gNB N + UE M
    NodeContainer ueNodes;
    NodeContainer mmWaveEnbNodes;
    NodeContainer lteEnbNodes;
    NodeContainer allEnbNodes;
    mmWaveEnbNodes.Create(nMmWaveEnbNodes);
    lteEnbNodes.Create(1);
    ueNodes.Create(nUes);
    allEnbNodes.Add(lteEnbNodes);
    allEnbNodes.Add(mmWaveEnbNodes);

    // 경희대 캠퍼스 좌표계 위치 (Kyunghee.xml과 동일 프레임)
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
    // LTE anchor is co-located with the first mmWave gNB when explicit positions
    // are supplied. It remains part of EN-DC but is hidden by the GUI relay.
    enbPositionAlloc->Add(configuredGnbPositions.empty()
                              ? Vector(-41.0, 37.0, GNB_Z)
                              : configuredGnbPositions.front().position);
    for (uint32_t i = 0; i < nMmWaveEnbNodes; ++i)
    {
        const Vector position = configuredGnbPositions.empty()
                                    ? Vector(-41.0 + 30.0 * i, 37.0 + 20.0 * i, GNB_Z)
                                    : configuredGnbPositions[i].position;
        enbPositionAlloc->Add(position);
        if (!configuredGnbPositions.empty())
        {
            NS_LOG_UNCOND("gNB '" << configuredGnbPositions[i].externalId << "' -> mmWave gNB "
                                   << i << " pos=(" << position.x << "," << position.y << ","
                                   << position.z << ")");
        }
    }

    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::SionnaMobilityModel", "Model",
                                 StringValue("Constant Position"));
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(allEnbNodes);

    MobilityHelper ueMobility;
    if (sumoTrace.empty())
    {
        Ptr<UniformDiscPositionAllocator> uePositionAlloc =
            CreateObject<UniformDiscPositionAllocator>();
        uePositionAlloc->SetX(30.0);
        uePositionAlloc->SetY(20.0);
        uePositionAlloc->SetZ(UE_Z);
        uePositionAlloc->SetRho(20.0);
        ueMobility.SetMobilityModel(
            "ns3::SionnaMobilityModel", "Model", StringValue("Random Walk"), "Mode",
            StringValue("Wall"), "Speed",
            StringValue("ns3::ConstantRandomVariable[Constant=1.4]"), "Direction",
            StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283184]"));
        ueMobility.SetPositionAllocator(uePositionAlloc);
    }
    else
    {
        Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
        for (uint32_t u = 0; u < nUes; ++u)
        {
            uePositionAlloc->Add(InitialPosition(ueTraces[u]));
        }
        ueMobility.SetMobilityModel("ns3::SionnaMobilityModel", "Model",
                                    StringValue("External"));
        ueMobility.SetPositionAllocator(uePositionAlloc);
    }
    ueMobility.Install(ueNodes);

    if (!sumoTrace.empty())
    {
        for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
        {
            Ptr<SionnaMobilityModel> mobility =
                ueNodes.Get(u)->GetObject<SionnaMobilityModel>();
            NS_ASSERT_MSG(mobility, "External UE does not have SionnaMobilityModel");
            mobility->SetActive(InitialActive(ueTraces[u]));
            NS_LOG_UNCOND("SUMO UE '" << ueTraces[u].externalId << "' -> UE " << u
                                       << ", node " << ueNodes.Get(u)->GetId());
            for (const auto& sample : ueTraces[u].samples)
            {
                if (sample.time < 0.0 || sample.time > simTime)
                {
                    continue;
                }
                Simulator::Schedule(Seconds(sample.time),
                                    [mobility, sample, u]() {
                                        mobility->SetPosition(sample.position);
                                        mobility->SetActive(sample.active);
                                        NS_LOG_UNCOND("[SUMO] t="
                                                      << Simulator::Now().GetSeconds() << " UE "
                                                      << u << " active=" << sample.active
                                                      << " pos=(" << sample.position.x << ","
                                                      << sample.position.y << ","
                                                      << sample.position.z << ")");
                                    });
            }
        }
    }

    // gNB/UE 디바이스 설치 -- 이 시점에 EnableSionna()로 세팅된 채널이 실제로 만들어짐
    NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice(lteEnbNodes);
    NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice(mmWaveEnbNodes);
    NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(mcUeDevs));
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    mmwaveHelper->AddX2Interface(lteEnbNodes, mmWaveEnbNodes);

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        mmwaveHelper->AttachToClosestEnb(mcUeDevs.Get(u), mmWaveEnbDevs, lteEnbDevs.Get(0));
    }

    // 노드 id <-> 역할 매핑을 시작 시 로그로 남김 (objN 대응표)
    NS_LOG_UNCOND("=== Node role map ===");
    NS_LOG_UNCOND("LTE anchor: node " << lteEnbNodes.Get(0)->GetId());
    for (uint32_t i = 0; i < mmWaveEnbNodes.GetN(); ++i)
    {
        NS_LOG_UNCOND("mmWave gNB " << i << ": node " << mmWaveEnbNodes.Get(i)->GetId());
    }
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        NS_LOG_UNCOND("UE " << i << ": node " << ueNodes.Get(i)->GetId());
    }

    // 간단한 다운링크 UDP 트래픽 (링크가 실제로 살아있는지 확인용)
    uint16_t dlPort = 1234;
    std::vector<Ptr<PacketSink>> downlinkSinks;
    downlinkSinks.reserve(ueNodes.GetN());
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        PacketSinkHelper dlPacketSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer sinkApp = dlPacketSinkHelper.Install(ueNodes.Get(u));
        downlinkSinks.push_back(DynamicCast<PacketSink>(sinkApp.Get(0)));
        sinkApp.Start(Seconds(trafficStart));
        sinkApp.Stop(Seconds(simTime));

        auto installClient = [&](double start, double stop) {
            start = std::max(start, trafficStart);
            stop = std::min(stop, simTime);
            if (stop <= start)
            {
                return;
            }
            UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
            dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(20)));
            dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
            dlClient.SetAttribute("PacketSize", UintegerValue(1024));
            ApplicationContainer clientApp = dlClient.Install(remoteHost);
            clientApp.Start(Seconds(start));
            clientApp.Stop(Seconds(stop));
        };

        if (sumoTrace.empty())
        {
            installClient(trafficStart, simTime);
        }
        else
        {
            bool active = false;
            double activeStart = 0.0;
            for (const auto& sample : ueTraces[u].samples)
            {
                if (sample.time < 0.0)
                {
                    active = sample.active;
                    if (active)
                    {
                        activeStart = 0.0;
                    }
                    continue;
                }
                if (sample.active && !active)
                {
                    active = true;
                    activeStart = sample.time;
                }
                else if (!sample.active && active)
                {
                    installClient(activeStart, sample.time);
                    active = false;
                }
            }
            if (active)
            {
                installClient(activeStart, simTime);
            }
        }
    }

    Simulator::Stop(Seconds(simTime));

    std::shared_ptr<SionnaHelper> sionnaHelper;
    if (enableSionna)
    {
        // 모든 노드/모빌리티가 설치된 뒤에 Sionna 서버와 세션을 연다.
        sionnaHelper = mmwaveHelper->GetSionnaHelper();
        NS_ASSERT_MSG(sionnaHelper, "EnableSionna() was not called");
        sionnaHelper->SetUeRadioStateProvider(
            [mcUeDevs](uint32_t nodeA,
                       uint32_t nodeB) -> std::optional<SionnaHelper::UeRadioState> {
                for (uint32_t i = 0; i < mcUeDevs.GetN(); ++i)
                {
                    Ptr<McUeNetDevice> ue = DynamicCast<McUeNetDevice>(mcUeDevs.Get(i));
                    if (!ue)
                    {
                        continue;
                    }
                    const uint32_t ueNodeId = ue->GetNode()->GetId();
                    if (ueNodeId != nodeA && ueNodeId != nodeB)
                    {
                        continue;
                    }

                    SionnaHelper::UeRadioState state;
                    state.nodeId = ueNodeId;
                    state.imsi = ue->GetImsi();
                    Ptr<LteUeRrc> rrc = ue->GetMmWaveRrc();
                    state.rnti = rrc ? rrc->GetRnti() : 0;
                    Ptr<SionnaMobilityModel> mobility =
                        ue->GetNode()->GetObject<SionnaMobilityModel>();
                    state.active = !mobility || mobility->IsActive();

                    Ptr<MmWaveEnbNetDevice> servingGnb = ue->GetMmWaveTargetEnb();
                    if (servingGnb)
                    {
                        state.servingCellId = servingGnb->GetCellId();
                        state.servingGnbNodeId = servingGnb->GetNode()->GetId();
                    }
                    return state;
                }
                return std::nullopt;
            });
        // frequency[MHz], bandwidth[MHz], fft_size, subcarrier_spacing[Hz] -- fft_size/spacing
        // chosen above so the returned CFR lines up 1:1 with ns-3's per-RB PSD bands.
        sionnaHelper->Configure(static_cast<int>(centerFrequency / 1e6),
                                static_cast<int>(bandwidth / 1e6), sionnaFftSize,
                                static_cast<int>(rbWidth));
        sionnaHelper->SetMode(SionnaHelper::MODE_P2MP);
        sionnaHelper->Start();
    }

    Simulator::Run();

    uint64_t totalDownlinkRxBytes = 0;
    for (uint32_t u = 0; u < downlinkSinks.size(); ++u)
    {
        NS_ASSERT_MSG(downlinkSinks[u], "Installed downlink application is not a PacketSink");
        const uint64_t rxBytes = downlinkSinks[u]->GetTotalRx();
        totalDownlinkRxBytes += rxBytes;
        NS_LOG_UNCOND("[DATA-PLANE] UE " << u << " received " << rxBytes << " bytes");
    }
    const double offeredDuration = simTime - trafficStart;
    const double aggregateThroughputMbps =
        totalDownlinkRxBytes * 8.0 / offeredDuration / 1e6;
    NS_LOG_UNCOND("[DATA-PLANE] aggregate received=" << totalDownlinkRxBytes
                                                      << " bytes, throughput="
                                                      << aggregateThroughputMbps << " Mbit/s");
    const bool trafficMissing = requireTraffic && totalDownlinkRxBytes == 0;

    if (sionnaHelper)
    {
        sionnaHelper->Destroy();
    }
    Simulator::Destroy();

    if (trafficMissing)
    {
        NS_LOG_UNCOND("[DATA-PLANE] FAIL: no downlink traffic reached any UE");
        return 2;
    }

    return 0;
}
