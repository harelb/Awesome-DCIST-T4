import {describe, expect, it} from "vitest";
import {leaseFromGuards, leaseRefusal, leaseRefusalOwner, leaseView, recentLeaseRefusal, shortOwner} from "../src/lib/lease";
import type {PlanStatus} from "../src/types";

describe("shortOwner", () => {
  it("shortens the tablet and our own client", () => {
    expect(shortOwner("bosdyn.android.spotapp 4.1.0 (Samsung SM-T)")).toBe("tablet");
    expect(shortOwner("bosdyn.android.spotapp")).toBe("tablet");
    expect(shortOwner("understanding-hamilton")).toBe("planner");
    expect(shortOwner("")).toBe("nobody");
    expect(shortOwner(undefined)).toBe("nobody");
    expect(shortOwner("bosdyn.choreographer host-7")).toBe("choreographer");
    expect(shortOwner("my_client")).toBe("my_client");
  });
});

describe("leaseView", () => {
  it("green when ours, amber with the holder, gray when unknown or stale", () => {
    expect(leaseView({known: true, owned: true, owner: "understanding-h", t: 100}, 101)).toMatchObject({tone: "ok", label: "lease: ours"});
    expect(leaseView({known: true, owned: false, owner: "bosdyn.android.spotapp x", t: 100}, 101)).toMatchObject({tone: "warn", label: "lease: tablet"});
    expect(leaseView({known: false, owned: false, owner: ""})).toMatchObject({tone: "off", label: "lease: unknown"});
    expect(leaseView(null)).toMatchObject({tone: "off"});
    expect(leaseView({known: true, owned: true, owner: "", t: 100}, 200)).toMatchObject({tone: "off", label: "lease: unknown"});
  });
  it("reads fleetlink guards", () => {
    expect(leaseFromGuards({lease_known: true, lease_owned: false, lease_owner: "bosdyn.android.spotapp", t: 5})).toEqual({known: true, owned: false, owner: "bosdyn.android.spotapp", t: 5});
    expect(leaseFromGuards({battery_known: true})).toBeNull();
  });
});

const st = (over: Partial<PlanStatus>): PlanStatus => ({plugin: "pddl", robot: "hamilton", ok: false, error: "", t: 1000, ...over});

describe("lease refusals", () => {
  it("parses the reason", () => {
    expect(leaseRefusalOwner("lease_not_owned(held by bosdyn.android.spotapp x)")).toBe("bosdyn.android.spotapp x");
    expect(leaseRefusalOwner("lease_not_owned")).toBe("");
    expect(leaseRefusalOwner("not_powered")).toBeNull();
  });
  it("uses structured refusals per robot", () => {
    const s = st({refusals: {hamilton: "lease_not_owned(held by bosdyn.android.spotapp)", euclid: "not_powered"}});
    expect(leaseRefusal(s, "hamilton")).toBe("bosdyn.android.spotapp");
    expect(leaseRefusal(s, "euclid")).toBeNull();
  });
  it("falls back to the error text", () => {
    const s = st({error: "plan refused by runtime dispatch guards: euclid: not_powered; hamilton: lease_not_owned(held by tablet x)"});
    expect(leaseRefusal(s, "hamilton")).toBe("tablet x");
    expect(leaseRefusal(s, "euclid")).toBeNull();
    // older omniplanner: no reason in the status at all
    expect(leaseRefusal(st({error: "plan refused by runtime dispatch guards"}), "hamilton")).toBeNull();
    expect(leaseRefusal(st({ok: true}), "hamilton")).toBeNull();
  });
  it("only the latest, recent status counts", () => {
    const refused = st({refusals: {hamilton: "lease_not_owned"}, t: 1000});
    expect(recentLeaseRefusal([refused], "hamilton", 1010)).toEqual({owner: "", t: 1000});
    expect(recentLeaseRefusal([refused, st({ok: true, t: 1005})], "hamilton", 1010)).toBeNull();
    expect(recentLeaseRefusal([refused], "hamilton", 5000)).toBeNull();
    expect(recentLeaseRefusal([refused], "hamilton", 1010, {since: 1001})).toBeNull();
    expect(recentLeaseRefusal(undefined, "hamilton", 1010)).toBeNull();
  });
});
