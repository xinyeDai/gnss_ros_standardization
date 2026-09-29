// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_ROS1_GNSS_CORE_HPP
#define GNSS_ROS_STANDARDIZATION_ROS1_GNSS_CORE_HPP

#include <ros/ros.h>

#include <gnss_ros_standardization/GnssEphemeris.h>
#include <gnss_ros_standardization/GlonassEphemeris.h>
#include <gnss_ros_standardization/GnssEphemerides.h>
#include <gnss_ros_standardization/GnssObservation.h>
#include <gnss_ros_standardization/GnssObservations.h>
#include <gnss_ros_standardization/GnssSolution.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

extern "C" {
#include "rtklib.h"
}

namespace gnss_ros1 {

inline std::string systemCode(int sys) {
  switch (sys) {
    case SYS_GPS: return "G";
    case SYS_GLO: return "R";
    case SYS_GAL: return "E";
    case SYS_QZS: return "J";
    case SYS_CMP: return "C";
    case SYS_IRN: return "I";
    case SYS_SBS: return "S";
    default: return "U";
  }
}

inline std::string satId(int sat) {
  char id[8] = {0};
  satno2id(sat, id);
  return std::string(id);
}

inline int satFromMsg(const std::string& satid, const std::string& sys, int prn) {
  if (!satid.empty()) {
    const int s = satid2no(satid.c_str());
    if (s > 0) return s;
  }
  int mask = 0;
  if (sys == "G") mask = SYS_GPS;
  else if (sys == "R") mask = SYS_GLO;
  else if (sys == "E") mask = SYS_GAL;
  else if (sys == "J") mask = SYS_QZS;
  else if (sys == "C") mask = SYS_CMP;
  else if (sys == "I") mask = SYS_IRN;
  else if (sys == "S") mask = SYS_SBS;
  else return 0;
  if (mask == SYS_QZS && prn >= 193 && prn <= 202) prn -= 192;
  return satno(mask, prn);
}

inline ros::Time gpstToUtcRosTime(gtime_t gpst) {
  const gtime_t utc = gpst2utc(gpst);
  ros::Time t;
  t.sec = static_cast<uint32_t>(utc.time);
  double frac = utc.sec;
  if (frac < 0.0) frac = 0.0;
  t.nsec = static_cast<uint32_t>(frac * 1e9);
  return t;
}

inline gnss_ros_standardization::GnssObservation obsToMsg(const obsd_t& o, int kf) {
  gnss_ros_standardization::GnssObservation m;
  int prn = 0;
  const int sys = satsys(o.sat, &prn);
  m.system = systemCode(sys);
  m.prn = static_cast<uint16_t>(prn);
  m.sat = o.sat;
  m.satid = satId(o.sat);
  if (o.code[kf]) {
    m.code = o.code[kf];
    const char* sig = code2obs(o.code[kf]);
    if (sig) m.code_str = sig;
  }
  m.p = o.P[kf];
  m.l = o.L[kf];
  m.d = o.D[kf];
  m.snr = static_cast<float>(o.SNR[kf]);
  m.lli = o.LLI[kf] & 0x07;
  return m;
}

inline gnss_ros_standardization::GnssEphemeris ephToMsg(const eph_t& e) {
  gnss_ros_standardization::GnssEphemeris m;
  int prn = 0;
  const int sys = satsys(e.sat, &prn);
  m.system = systemCode(sys);
  m.prn = static_cast<uint16_t>(prn);
  m.satid = satId(e.sat);

  int w = 0;
  m.toe = time2gpst(e.toe, &w);
  m.week = static_cast<uint32_t>(w);
  m.toc = time2gpst(e.toc, &w);
  m.ttr = time2gpst(e.ttr, &w);
  m.toes = e.toes;

  m.a = e.A; m.e = e.e; m.i0 = e.i0; m.omg0 = e.OMG0;
  m.omg = e.omg; m.m0 = e.M0; m.deln = e.deln;
  m.omgd = e.OMGd; m.idot = e.idot;
  m.crc = e.crc; m.crs = e.crs; m.cuc = e.cuc;
  m.cus = e.cus; m.cic = e.cic; m.cis = e.cis;
  m.f0 = e.f0; m.f1 = e.f1; m.f2 = e.f2;
  m.tgd.push_back(e.tgd[0]);
  m.tgd.push_back(e.tgd[1]);
  m.iode = static_cast<uint8_t>(e.iode);
  m.iodc = static_cast<uint16_t>(e.iodc);
  if (m.system == "C" && e.iode > 31 && e.iodc <= 31) {
    m.iode = static_cast<uint8_t>(e.iodc);
  }
  m.svh = static_cast<uint16_t>(e.svh);
  m.sva = static_cast<uint8_t>(e.sva);
  m.code = static_cast<uint16_t>(e.code);
  m.flag = static_cast<uint8_t>(e.flag);
  m.fit = static_cast<uint8_t>(e.fit);
  return m;
}

inline gnss_ros_standardization::GlonassEphemeris gephToMsg(const geph_t& g) {
  gnss_ros_standardization::GlonassEphemeris m;
  m.system = "R";
  int prn = 0;
  satsys(g.sat, &prn);
  m.prn = static_cast<uint16_t>(prn);
  m.satid = satId(g.sat);
  m.frq = g.frq;
  int w = 0;
  m.toe = time2gpst(utc2gpst(g.toe), &w);
  m.week = static_cast<uint32_t>(w);
  m.tof = time2gpst(utc2gpst(g.tof), &w);
  m.pos.assign(g.pos, g.pos + 3);
  m.vel.assign(g.vel, g.vel + 3);
  m.acc.assign(g.acc, g.acc + 3);
  m.iode = static_cast<uint8_t>(g.iode);
  m.svh = static_cast<uint8_t>(g.svh);
  m.age = static_cast<uint8_t>(g.age);
  m.gamn = g.gamn;
  m.taun = g.taun;
  m.dtaun = g.dtaun;
  return m;
}

inline gtime_t adjWeek(gtime_t ref, int week, double tow) {
  gtime_t t = gpst2time(week, tow);
  const double dt = timediff(t, ref);
  if (dt < -302400.0) t = timeadd(t, 604800.0);
  else if (dt > 302400.0) t = timeadd(t, -604800.0);
  return t;
}

inline eph_t msgToEph(const gnss_ros_standardization::GnssEphemeris& m) {
  eph_t e{};
  e.sat = satFromMsg(m.satid, m.system, m.prn);
  const int w = static_cast<int>(m.week);
  e.toe = gpst2time(w, m.toe);
  e.toc = adjWeek(e.toe, w, m.toc);
  e.ttr = adjWeek(e.toe, w, m.ttr);
  int final_week = 0;
  if (satsys(e.sat, NULL) == SYS_CMP) {
    if (e.toe.time != 0) time2bdt(e.toe, &final_week);
  } else if (e.toe.time != 0) {
    time2gpst(e.toe, &final_week);
  }
  e.week = final_week;

  e.A = m.a; e.e = m.e; e.i0 = m.i0; e.OMG0 = m.omg0;
  e.omg = m.omg; e.M0 = m.m0; e.deln = m.deln;
  e.OMGd = m.omgd; e.idot = m.idot;
  e.crc = m.crc; e.crs = m.crs; e.cuc = m.cuc;
  e.cus = m.cus; e.cic = m.cic; e.cis = m.cis;
  e.f0 = m.f0; e.f1 = m.f1; e.f2 = m.f2;
  e.tgd[0] = m.tgd.size() > 0 ? m.tgd[0] : 0.0;
  e.tgd[1] = m.tgd.size() > 1 ? m.tgd[1] : 0.0;
  e.iode = m.iode; e.iodc = m.iodc;
  if (m.system == "C" && e.iode > 31 && e.iodc <= 31) e.iode = e.iodc;
  e.svh = m.svh; e.sva = m.sva; e.code = m.code;
  e.flag = m.flag; e.fit = m.fit; e.toes = m.toes;
  return e;
}

inline geph_t msgToGeph(const gnss_ros_standardization::GlonassEphemeris& m) {
  geph_t g{};
  g.sat = satFromMsg(m.satid, m.system, m.prn);
  const int w = static_cast<int>(m.week);
  g.toe = gpst2utc(gpst2time(w, m.toe));
  g.tof = gpst2utc(gpst2time(w, m.tof));
  g.frq = static_cast<signed char>(m.frq);
  for (int i = 0; i < 3; ++i) {
    g.pos[i] = m.pos.size() > static_cast<size_t>(i) ? m.pos[i] : 0.0;
    g.vel[i] = m.vel.size() > static_cast<size_t>(i) ? m.vel[i] : 0.0;
    g.acc[i] = m.acc.size() > static_cast<size_t>(i) ? m.acc[i] : 0.0;
  }
  g.iode = m.iode; g.svh = m.svh; g.age = m.age;
  g.gamn = m.gamn; g.taun = m.taun; g.dtaun = m.dtaun;
  return g;
}

inline bool sameEph(const eph_t& a, const eph_t& b) {
  return a.sat == b.sat && a.code == b.code &&
         a.iode == b.iode && a.iodc == b.iodc &&
         std::fabs(timediff(a.toe, b.toe)) < 1e-6;
}

inline bool sameGeph(const geph_t& a, const geph_t& b) {
  return a.sat == b.sat && a.iode == b.iode &&
         std::fabs(timediff(a.toe, b.toe)) < 1e-6 &&
         std::fabs(timediff(a.tof, b.tof)) < 1e-6;
}

inline void upsertEph(nav_t& nav, const eph_t& e) {
  if (e.sat <= 0 || e.sat > MAXSAT) return;
  for (int i = 0; i < nav.n; ++i) {
    if (sameEph(nav.eph[i], e)) {
      nav.eph[i] = e;
      return;
    }
  }
  if (nav.n >= nav.nmax) {
    const int newmax = nav.nmax == 0 ? 16 : nav.nmax * 2;
    eph_t* p = static_cast<eph_t*>(std::realloc(nav.eph, sizeof(eph_t) * newmax));
    if (!p) return;
    nav.eph = p;
    nav.nmax = newmax;
  }
  nav.eph[nav.n++] = e;
}

inline void upsertGeph(nav_t& nav, const geph_t& g) {
  if (g.sat <= 0 || g.sat > MAXSAT) return;
  for (int i = 0; i < nav.ng; ++i) {
    if (sameGeph(nav.geph[i], g)) {
      nav.geph[i] = g;
      return;
    }
  }
  if (nav.ng >= nav.ngmax) {
    const int newmax = nav.ngmax == 0 ? 16 : nav.ngmax * 2;
    geph_t* p = static_cast<geph_t*>(std::realloc(nav.geph, sizeof(geph_t) * newmax));
    if (!p) return;
    nav.geph = p;
    nav.ngmax = newmax;
  }
  nav.geph[nav.ng++] = g;
}

inline void ingestEphemerides(nav_t& nav,
                              const gnss_ros_standardization::GnssEphemerides& msg) {
  for (size_t i = 0; i < msg.gnss_ephemeris.size(); ++i)
    upsertEph(nav, msgToEph(msg.gnss_ephemeris[i]));
  for (size_t i = 0; i < msg.glonass_ephemeris.size(); ++i)
    upsertGeph(nav, msgToGeph(msg.glonass_ephemeris[i]));
}

struct FrequencyMask {
  bool l1{true};
  bool l2{true};
  bool l5{true};
};

inline bool isPrimaryCode(int sys, uint8_t code, int idx) {
  if (sys == SYS_CMP && idx == 0 && code != CODE_L2I) return false;
  return true;
}

inline obsd_t convertObs(const gnss_ros_standardization::GnssObservation& m,
                         gtime_t t, int rcv, const FrequencyMask& mask) {
  obsd_t o{};
  const int sat = satid2no(m.satid.c_str());
  if (sat <= 0 || sat > MAXSAT) return o;
  int prn = 0;
  const int sys = satsys(sat, &prn);
  const int idx = code2idx(sys, m.code);
  if (idx < 0 || idx >= NFREQ) return o;
  if (!isPrimaryCode(sys, m.code, idx)) return o;
  if (idx == 0 && !mask.l1) return o;
  if (idx == 1 && !mask.l2) return o;
  if (idx == 2 && !mask.l5) return o;

  o.time = t;
  o.sat = static_cast<uint8_t>(sat);
  o.rcv = static_cast<uint8_t>(rcv);
  o.P[idx] = m.p; o.L[idx] = m.l; o.D[idx] = static_cast<float>(m.d);
  o.SNR[idx] = static_cast<float>(m.snr);
  o.LLI[idx] = m.lli; o.code[idx] = m.code;
  return o;
}

inline std::vector<obsd_t> buildObsEpoch(
    const gnss_ros_standardization::GnssObservations& rover,
    const gnss_ros_standardization::GnssObservations* base,
    const FrequencyMask& mask) {
  std::map<int, obsd_t> map;
  const gtime_t tr = gpst2time(static_cast<int>(rover.week), rover.tow);

  const auto merge = [&](const gnss_ros_standardization::GnssObservation& m,
                         gtime_t t, int rcv, std::map<int, obsd_t>& dst) {
    const obsd_t o = convertObs(m, t, rcv, mask);
    if (o.sat == 0) return;
    const int key = (rcv << 16) | o.sat;
    std::map<int, obsd_t>::iterator it = dst.find(key);
    if (it == dst.end()) {
      dst[key] = o;
      return;
    }
    obsd_t& cur = it->second;
    int prn = 0;
    const int sys = satsys(o.sat, &prn);
    for (int i = 0; i < NFREQ; ++i) {
      if (o.code[i] == 0) continue;
      if (cur.code[i] != 0 &&
          getcodepri(sys, o.code[i], "") <= getcodepri(sys, cur.code[i], "")) continue;
      cur.P[i] = o.P[i]; cur.L[i] = o.L[i]; cur.D[i] = o.D[i];
      cur.SNR[i] = o.SNR[i]; cur.LLI[i] = o.LLI[i]; cur.code[i] = o.code[i];
    }
  };

  for (size_t i = 0; i < rover.observations.size(); ++i)
    merge(rover.observations[i], tr, 1, map);

  if (base) {
    const gtime_t tb = gpst2time(static_cast<int>(base->week), base->tow);
    for (size_t i = 0; i < base->observations.size(); ++i)
      merge(base->observations[i], tb, 2, map);
  }

  std::vector<obsd_t> out;
  out.reserve(map.size());
  for (std::map<int, obsd_t>::const_iterator it = map.begin(); it != map.end(); ++it)
    out.push_back(it->second);
  std::sort(out.begin(), out.end(), [](const obsd_t& a, const obsd_t& b) {
    if (a.rcv != b.rcv) return a.rcv < b.rcv;
    return a.sat < b.sat;
  });
  return out;
}

inline void rotateCovariance(const double in[9], double lat, double lon, double out[9]) {
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double sL = std::sin(lon), cL = std::cos(lon);
  const double R[9] = {
    -sL, cL, 0.0,
    -sl*cL, -sl*sL, cl,
    cl*cL, cl*sL, sl
  };
  double T[9] = {0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        T[3*i+j] += in[3*i+k] * R[3*j+k];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      out[3*i+j] = 0.0;
      for (int k = 0; k < 3; ++k)
        out[3*i+j] += R[3*i+k] * T[3*k+j];
    }
}

struct Dops {
  double gdop{0.0}, pdop{0.0}, hdop{0.0}, vdop{0.0};
};

inline Dops calculateDops(const ssat_t* ssat, int nmax, double elmin) {
  Dops d;
  double azel[MAXSAT * 2] = {0};
  int n = 0;
  for (int i = 0; i < nmax && n < MAXSAT; ++i) {
    if (!ssat[i].vs) continue;
    azel[2*n] = ssat[i].azel[0];
    azel[2*n+1] = ssat[i].azel[1];
    ++n;
  }
  if (n >= 4) {
    double dop[4] = {0};
    dops(n, azel, elmin, dop);
    d.gdop = dop[0]; d.pdop = dop[1]; d.hdop = dop[2]; d.vdop = dop[3];
  }
  return d;
}

inline void fillCovarianceFromSol(const sol_t& sol,
                                  gnss_ros_standardization::GnssSolution& out,
                                  const double llh[3]) {
  out.pos_cov_ecef[0] = sol.qr[0]; out.pos_cov_ecef[4] = sol.qr[1];
  out.pos_cov_ecef[8] = sol.qr[2];
  out.pos_cov_ecef[1] = out.pos_cov_ecef[3] = sol.qr[3];
  out.pos_cov_ecef[5] = out.pos_cov_ecef[7] = sol.qr[4];
  out.pos_cov_ecef[2] = out.pos_cov_ecef[6] = sol.qr[5];

  out.vel_cov_ecef[0] = sol.qv[0]; out.vel_cov_ecef[4] = sol.qv[1];
  out.vel_cov_ecef[8] = sol.qv[2];
  out.vel_cov_ecef[1] = out.vel_cov_ecef[3] = sol.qv[3];
  out.vel_cov_ecef[5] = out.vel_cov_ecef[7] = sol.qv[4];
  out.vel_cov_ecef[2] = out.vel_cov_ecef[6] = sol.qv[5];

  double qpos[9], qvel[9];
  for (int i = 0; i < 9; ++i) {
    qpos[i] = out.pos_cov_ecef[i];
    qvel[i] = out.vel_cov_ecef[i];
  }
  double enu[9], venu[9];
  rotateCovariance(qpos, llh[0], llh[1], enu);
  rotateCovariance(qvel, llh[0], llh[1], venu);
  for (int i = 0; i < 9; ++i) {
    out.pos_enu_cov[i] = enu[i];
    out.vel_enu_cov[i] = venu[i];
  }
}

inline gnss_ros_standardization::GnssSolution makeSolution(
    const sol_t& sol, const ssat_t* ssat, double elmin,
    const ros::Time& stamp, const double origin_ecef[3]) {
  gnss_ros_standardization::GnssSolution out;
  out.header.stamp = stamp;
  out.header.frame_id = "gnss_link";

  int week = 0;
  out.time_tow = time2gpst(sol.time, &week);
  out.time_week = static_cast<uint32_t>(week);
  out.solution_source = gnss_ros_standardization::GnssSolution::SOLUTION_SOURCE_COMPUTED;
  out.status = static_cast<uint8_t>(sol.stat);
  out.num_sats = static_cast<uint8_t>(sol.ns);
  out.ratio = static_cast<float>(sol.ratio);
  out.age_diff = static_cast<float>(sol.age);

  const Dops dop = calculateDops(ssat, MAXSAT, elmin);
  out.gdop = dop.gdop; out.pdop = dop.pdop; out.hdop = dop.hdop; out.vdop = dop.vdop;

  double llh[3] = {0};
  ecef2pos(sol.rr, llh);
  out.latitude = llh[0] * R2D;
  out.longitude = llh[1] * R2D;
  out.altitude = llh[2];

  out.pos_ecef.x = sol.rr[0]; out.pos_ecef.y = sol.rr[1]; out.pos_ecef.z = sol.rr[2];
  out.vel_ecef.x = sol.rr[3]; out.vel_ecef.y = sol.rr[4]; out.vel_ecef.z = sol.rr[5];

  out.pos_enu_org_ecef.x = origin_ecef[0];
  out.pos_enu_org_ecef.y = origin_ecef[1];
  out.pos_enu_org_ecef.z = origin_ecef[2];

  if (norm(origin_ecef, 3) > 0.0) {
    double org_llh[3] = {0};
    ecef2pos(origin_ecef, org_llh);
    double dxyz[3] = {sol.rr[0]-origin_ecef[0],
                      sol.rr[1]-origin_ecef[1],
                      sol.rr[2]-origin_ecef[2]};
    double enu[3] = {0};
    ecef2enu(org_llh, dxyz, enu);
    out.pos_enu.x = enu[0]; out.pos_enu.y = enu[1]; out.pos_enu.z = enu[2];
  }

  double vecef[3] = {sol.rr[3], sol.rr[4], sol.rr[5]};
  double venu[3] = {0};
  ecef2enu(llh, vecef, venu);
  out.vel_enu.x = venu[0]; out.vel_enu.y = venu[1]; out.vel_enu.z = venu[2];

  fillCovarianceFromSol(sol, out, llh);
  return out;
}

}  // namespace gnss_ros1
#endif
