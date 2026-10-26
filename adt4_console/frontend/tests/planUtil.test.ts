import {describe, expect, it} from "vitest";
import {distinctPoints} from "../src/viewer/planUtil";

describe("distinctPoints", () => {
  it("drops consecutive repeats (zero-length segments stop drei's Line drawing)", () => {
    const a: [number, number, number] = [0, 0, 0];
    const b: [number, number, number] = [1, 0, 0];
    expect(distinctPoints([a, a, b, b, a])).toEqual([a, b, a]);
    expect(distinctPoints([])).toEqual([]);
  });
});
