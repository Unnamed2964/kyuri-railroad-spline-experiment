import { useEffect, useMemo, useRef, useState } from 'react';

type Point = {
  x: number;
  y: number;
};

type Pose = {
  point: Point;
  heading: number;
};

type CurvePoint = Point & {
  theta: number;
};

type Viewport = {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
};

type DragTarget = 'start-point' | 'end-point' | 'start-handle' | 'end-handle';

type StraightPadSolverResult = {
  status: 'ok' | 'approx' | 'straight-only' | 'invalid';
  radius: number;
  transitionLength: number;
  circleLength: number;
  totalLength: number;
  circleAngle: number;
  headingDelta: number;
  leadInLength: number;
  leadOutLength: number;
  minRadius: number;
  turnSign: 1 | -1;
  positionError: number;
  entrySampleCount: number;
  circleSampleCount: number;
  coreStart: Point;
  coreEnd: CurvePoint;
  solvedEnd: Point;
  path: CurvePoint[];
};

const CANVAS_HEIGHT = 720;
const HANDLE_LENGTH = 84;
const HANDLE_HIT_RADIUS = 14;
const POINT_HIT_RADIUS = 12;
const WORLD_BOUNDS = {
  minX: -220,
  maxX: 1220,
  minY: -220,
  maxY: 860,
};

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

function distance(a: Point, b: Point) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function rotate(point: Point, angle: number) {
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  return {
    x: point.x * cosine - point.y * sine,
    y: point.x * sine + point.y * cosine,
  };
}

function subtract(a: Point, b: Point) {
  return { x: a.x - b.x, y: a.y - b.y };
}

function wrapAngle(angle: number) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function angleToDegrees(angle: number) {
  return (angle * 180) / Math.PI;
}

function formatValue(value: number, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : '∞';
}

function getHandlePoint(pose: Pose) {
  return {
    x: pose.point.x + Math.cos(pose.heading) * HANDLE_LENGTH,
    y: pose.point.y + Math.sin(pose.heading) * HANDLE_LENGTH,
  };
}

function extendPointToBounds(origin: Point, heading: number, directionSign: 1 | -1, bounds: Viewport) {
  const direction = {
    x: Math.cos(heading) * directionSign,
    y: Math.sin(heading) * directionSign,
  };
  let maxDistance = Number.POSITIVE_INFINITY;

  if (Math.abs(direction.x) > 1e-9) {
    const xDistance =
      direction.x > 0
        ? (bounds.maxX - origin.x) / direction.x
        : (bounds.minX - origin.x) / direction.x;
    if (xDistance >= 0) {
      maxDistance = Math.min(maxDistance, xDistance);
    }
  }

  if (Math.abs(direction.y) > 1e-9) {
    const yDistance =
      direction.y > 0
        ? (bounds.maxY - origin.y) / direction.y
        : (bounds.minY - origin.y) / direction.y;
    if (yDistance >= 0) {
      maxDistance = Math.min(maxDistance, yDistance);
    }
  }

  if (!Number.isFinite(maxDistance)) {
    return origin;
  }

  return {
    x: origin.x + direction.x * maxDistance,
    y: origin.y + direction.y * maxDistance,
  };
}

function createProjector(viewport: Viewport, width: number, height: number) {
  const worldWidth = Math.max(viewport.maxX - viewport.minX, 1);
  const worldHeight = Math.max(viewport.maxY - viewport.minY, 1);
  const scale = Math.min(width / worldWidth, height / worldHeight);
  const offsetX = (width - worldWidth * scale) / 2;
  const offsetY = (height - worldHeight * scale) / 2;

  return {
    project(point: Point) {
      return {
        x: offsetX + (point.x - viewport.minX) * scale,
        y: height - offsetY - (point.y - viewport.minY) * scale,
      };
    },
    unproject(point: Point) {
      return {
        x: viewport.minX + (point.x - offsetX) / scale,
        y: viewport.minY + (height - offsetY - point.y) / scale,
      };
    },
  };
}

function simulateSymmetricCurve(
  parameterA: number,
  radius: number,
  headingDelta: number,
  turnSign: 1 | -1,
  returnSamples: boolean,
) {
  const transitionLength = (parameterA * parameterA) / radius;
  const transitionAngle = (transitionLength * transitionLength) / (2 * parameterA * parameterA);
  const rawCircleAngle = Math.abs(headingDelta) - 2 * transitionAngle;

  if (rawCircleAngle < -1e-6) {
    return null;
  }

  const circleAngle = Math.max(0, rawCircleAngle);
  const circleLength = Math.max(0, radius * circleAngle);
  const totalLength = transitionLength * 2 + circleLength;
  const segmentCount = Math.max(240, Math.ceil(totalLength / 3));
  const entrySteps = Math.max(24, Math.round((segmentCount * transitionLength) / totalLength));
  const circleSteps = Math.max(16, Math.round((segmentCount * circleLength) / totalLength));
  const exitSteps = entrySteps;
  let x = 0;
  let y = 0;
  let theta = 0;
  const samples: CurvePoint[] = returnSamples ? [{ x: 0, y: 0, theta: 0 }] : [];
  let entrySampleCount = returnSamples ? 1 : 0;
  let circleSampleCount = 0;

  const advance = (length: number, steps: number, curvatureAt: (index: number, deltaS: number) => number) => {
    if (length <= 1e-9 || steps <= 0) {
      return;
    }

    const deltaS = length / steps;

    for (let index = 0; index < steps; index += 1) {
      const curvature = curvatureAt(index, deltaS);
      const deltaTheta = curvature * deltaS;
      const midTheta = theta + deltaTheta * 0.5;
      x += Math.cos(midTheta) * deltaS;
      y += Math.sin(midTheta) * deltaS;
      theta += deltaTheta;

      if (returnSamples) {
        samples.push({ x, y, theta });
      }
    }
  };

  advance(transitionLength, entrySteps, (index, deltaS) => {
    const sMid = (index + 0.5) * deltaS;
    return turnSign * (sMid / (parameterA * parameterA));
  });

  if (returnSamples) {
    entrySampleCount = samples.length;
  }

  advance(circleLength, circleSteps, () => turnSign / radius);

  if (returnSamples) {
    circleSampleCount = samples.length - entrySampleCount;
  }

  advance(transitionLength, exitSteps, (index, deltaS) => {
    const sMid = (index + 0.5) * deltaS;
    return turnSign * ((transitionLength - sMid) / (parameterA * parameterA));
  });

  return {
    transitionLength,
    circleLength,
    totalLength,
    circleAngle,
    entrySampleCount,
    circleSampleCount,
    end: { x, y, theta },
    samples,
  };
}

function solveStraightPadding(
  target: Point,
  localCurveEnd: Point,
  headingDelta: number,
): { feasible: boolean; leadInLength: number; leadOutLength: number; residualError: number } {
  const residual = {
    x: target.x - localCurveEnd.x,
    y: target.y - localCurveEnd.y,
  };
  const sinDelta = Math.sin(headingDelta);
  const cosDelta = Math.cos(headingDelta);
  const tolerance = 1e-5;

  if (Math.abs(sinDelta) > 1e-5) {
    const leadOutLength = residual.y / sinDelta;
    const leadInLength = residual.x - leadOutLength * cosDelta;
    const feasible = leadInLength >= -tolerance && leadOutLength >= -tolerance;
    return {
      feasible,
      leadInLength: Math.max(0, leadInLength),
      leadOutLength: Math.max(0, leadOutLength),
      residualError: 0,
    };
  }

  if (Math.abs(residual.y) > 1e-3) {
    return {
      feasible: false,
      leadInLength: 0,
      leadOutLength: 0,
      residualError: Math.abs(residual.y),
    };
  }

  if (cosDelta > 0) {
    return {
      feasible: residual.x >= -tolerance,
      leadInLength: Math.max(0, residual.x),
      leadOutLength: 0,
      residualError: 0,
    };
  }

  return {
    feasible: true,
    leadInLength: Math.max(0, residual.x),
    leadOutLength: Math.max(0, -residual.x),
    residualError: 0,
  };
}

function solveMinRadiusWithStraightPads(startPose: Pose, endPose: Pose, parameterA: number): StraightPadSolverResult {
  const headingDelta = wrapAngle(endPose.heading - startPose.heading);
  const relativeTarget = rotate(subtract(endPose.point, startPose.point), -startPose.heading);
  const chordLength = Math.max(distance(startPose.point, endPose.point), 1);

  if (Math.abs(headingDelta) < 0.01) {
    const alignedY = Math.abs(relativeTarget.y);
    if (alignedY < 1e-3 && relativeTarget.x >= 0) {
      return {
        status: 'straight-only',
        radius: Number.POSITIVE_INFINITY,
        transitionLength: 0,
        circleLength: 0,
        totalLength: relativeTarget.x,
        circleAngle: 0,
        headingDelta,
        leadInLength: relativeTarget.x,
        leadOutLength: 0,
        minRadius: Number.POSITIVE_INFINITY,
        turnSign: 1,
        positionError: 0,
        entrySampleCount: 0,
        circleSampleCount: 0,
        coreStart: startPose.point,
        coreEnd: { x: startPose.point.x, y: startPose.point.y, theta: startPose.heading },
        solvedEnd: endPose.point,
        path: [],
      };
    }
  }

  const turnSign = Math.sign(headingDelta) >= 0 ? 1 : -1;
  const minRadius = parameterA / Math.sqrt(Math.max(Math.abs(headingDelta), 1e-6));
  const lowerBound = minRadius * 1.0005;
  const upperBound = Math.max(lowerBound * 36, chordLength * 12, parameterA * 18);
  const sampleCount = 360;
  let lastFeasibleRadius: number | null = null;
  let firstInfeasibleAfterFeasible: number | null = null;

  const isFeasible = (radius: number) => {
    const simulation = simulateSymmetricCurve(parameterA, radius, headingDelta, turnSign, false);
    if (!simulation) {
      return false;
    }

    return solveStraightPadding(relativeTarget, simulation.end, headingDelta).feasible;
  };

  for (let index = 0; index <= sampleCount; index += 1) {
    const ratio = index / sampleCount;
    const radius = lowerBound * Math.exp(Math.log(upperBound / lowerBound) * ratio);
    if (isFeasible(radius)) {
      lastFeasibleRadius = radius;
      continue;
    }

    if (lastFeasibleRadius !== null) {
      firstInfeasibleAfterFeasible = radius;
      break;
    }
  }

  if (lastFeasibleRadius === null) {
    return {
      status: 'invalid',
      radius: Number.POSITIVE_INFINITY,
      transitionLength: 0,
      circleLength: 0,
      totalLength: 0,
      circleAngle: 0,
      headingDelta,
      leadInLength: 0,
      leadOutLength: 0,
      minRadius,
      turnSign,
      positionError: Number.POSITIVE_INFINITY,
      entrySampleCount: 0,
      circleSampleCount: 0,
      coreStart: startPose.point,
      coreEnd: { x: startPose.point.x, y: startPose.point.y, theta: startPose.heading },
      solvedEnd: startPose.point,
      path: [],
    };
  }

  if (firstInfeasibleAfterFeasible === null) {
    firstInfeasibleAfterFeasible = upperBound;
  }

  let left = lastFeasibleRadius;
  let right = firstInfeasibleAfterFeasible;
  for (let iteration = 0; iteration < 36; iteration += 1) {
    const middle = (left + right) * 0.5;
    if (isFeasible(middle)) {
      left = middle;
    } else {
      right = middle;
    }
  }

  const bestRadius = left;
  const simulation = simulateSymmetricCurve(parameterA, bestRadius, headingDelta, turnSign, true);
  if (!simulation) {
    return {
      status: 'invalid',
      radius: Number.POSITIVE_INFINITY,
      transitionLength: 0,
      circleLength: 0,
      totalLength: 0,
      circleAngle: 0,
      headingDelta,
      leadInLength: 0,
      leadOutLength: 0,
      minRadius,
      turnSign,
      positionError: Number.POSITIVE_INFINITY,
      entrySampleCount: 0,
      circleSampleCount: 0,
      coreStart: startPose.point,
      coreEnd: { x: startPose.point.x, y: startPose.point.y, theta: startPose.heading },
      solvedEnd: startPose.point,
      path: [],
    };
  }

  const padding = solveStraightPadding(relativeTarget, simulation.end, headingDelta);
  const coreStart = {
    x: startPose.point.x + Math.cos(startPose.heading) * padding.leadInLength,
    y: startPose.point.y + Math.sin(startPose.heading) * padding.leadInLength,
  };
  const path = simulation.samples.map((sample) => {
    const rotated = rotate(sample, startPose.heading);
    return {
      x: rotated.x + coreStart.x,
      y: rotated.y + coreStart.y,
      theta: wrapAngle(sample.theta + startPose.heading),
    };
  });
  const coreEnd = path[path.length - 1] ?? { x: coreStart.x, y: coreStart.y, theta: startPose.heading };
  const solvedEnd = {
    x: coreEnd.x + Math.cos(endPose.heading) * padding.leadOutLength,
    y: coreEnd.y + Math.sin(endPose.heading) * padding.leadOutLength,
  };

  return {
    status: padding.residualError < 1e-4 ? 'ok' : 'approx',
    radius: bestRadius,
    transitionLength: simulation.transitionLength,
    circleLength: simulation.circleLength,
    totalLength: simulation.totalLength + padding.leadInLength + padding.leadOutLength,
    circleAngle: simulation.circleAngle,
    headingDelta,
    leadInLength: padding.leadInLength,
    leadOutLength: padding.leadOutLength,
    minRadius,
    turnSign,
    positionError: distance(solvedEnd, endPose.point),
    entrySampleCount: simulation.entrySampleCount,
    circleSampleCount: simulation.circleSampleCount,
    coreStart,
    coreEnd,
    solvedEnd,
    path,
  };
}

function drawPathSegment(
  context: CanvasRenderingContext2D,
  projector: ReturnType<typeof createProjector>,
  points: CurvePoint[],
  strokeStyle: string,
) {
  if (points.length < 2) {
    return;
  }

  context.strokeStyle = strokeStyle;
  context.lineWidth = 1.8;
  context.beginPath();
  points.forEach((point, index) => {
    const screenPoint = projector.project(point);
    if (index === 0) {
      context.moveTo(screenPoint.x, screenPoint.y);
    } else {
      context.lineTo(screenPoint.x, screenPoint.y);
    }
  });
  context.stroke();
}

export function TwoPointStraightPadSolverPage() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const canvasHostRef = useRef<HTMLDivElement | null>(null);
  const [canvasWidth, setCanvasWidth] = useState(960);
  const [parameterA, setParameterA] = useState(120);
  const [startPose, setStartPose] = useState<Pose>({
    point: { x: 160, y: 520 },
    heading: -0.35,
  });
  const [endPose, setEndPose] = useState<Pose>({
    point: { x: 900, y: 220 },
    heading: 2.64,
  });
  const [dragTarget, setDragTarget] = useState<DragTarget | null>(null);

  useEffect(() => {
    if (!canvasHostRef.current) {
      return undefined;
    }

    const observer = new ResizeObserver((entries) => {
      const nextWidth = Math.max(320, Math.floor(entries[0]?.contentRect.width ?? 960));
      setCanvasWidth(nextWidth);
    });

    observer.observe(canvasHostRef.current);
    return () => observer.disconnect();
  }, []);

  const solverResult = useMemo(
    () => solveMinRadiusWithStraightPads(startPose, endPose, parameterA),
    [endPose, parameterA, startPose],
  );
  const viewport: Viewport = WORLD_BOUNDS;
  const startHandle = getHandlePoint(startPose);
  const endHandle = getHandlePoint(endPose);
  const extendedStartPoint = extendPointToBounds(startPose.point, startPose.heading, -1, viewport);
  const extendedEndPoint = extendPointToBounds(endPose.point, endPose.heading, 1, viewport);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(320, Math.floor(canvasWidth));
    const height = CANVAS_HEIGHT;
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;

    const context = canvas.getContext('2d');
    if (!context) {
      return;
    }

    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, width, height);
    context.fillStyle = '#ffffff';
    context.fillRect(0, 0, width, height);

    const projector = createProjector(viewport, width, height);
    const startPointOnScreen = projector.project(startPose.point);
    const endPointOnScreen = projector.project(endPose.point);
    const startHandleOnScreen = projector.project(startHandle);
    const endHandleOnScreen = projector.project(endHandle);
    const coreStartOnScreen = projector.project(solverResult.coreStart);
    const coreEndOnScreen = projector.project(solverResult.coreEnd);
    const extendedStartOnScreen = projector.project(extendedStartPoint);
    const extendedEndOnScreen = projector.project(extendedEndPoint);

    context.strokeStyle = '#d4d4d8';
    context.lineWidth = 1;
    context.strokeRect(0.5, 0.5, width - 1, height - 1);

    context.fillStyle = '#eef2f7';
    for (let x = 80; x < width; x += 80) {
      context.fillRect(x, 0, 1, height);
    }
    for (let y = 80; y < height; y += 80) {
      context.fillRect(0, y, width, 1);
    }

    context.strokeStyle = '#16a34a';
    context.lineWidth = 1.25;
    context.beginPath();
    context.moveTo(extendedStartOnScreen.x, extendedStartOnScreen.y);
    context.lineTo(coreStartOnScreen.x, coreStartOnScreen.y);
    context.moveTo(coreEndOnScreen.x, coreEndOnScreen.y);
    context.lineTo(extendedEndOnScreen.x, extendedEndOnScreen.y);
    context.stroke();

    if (solverResult.path.length > 1) {
      const entryEnd = Math.max(1, solverResult.entrySampleCount);
      const circleEnd = Math.min(
        solverResult.path.length,
        solverResult.entrySampleCount + solverResult.circleSampleCount,
      );
      drawPathSegment(context, projector, solverResult.path.slice(0, entryEnd), '#dc2626');
      drawPathSegment(
        context,
        projector,
        solverResult.path.slice(Math.max(0, entryEnd - 1), Math.max(circleEnd, entryEnd)),
        '#eab308',
      );
      drawPathSegment(context, projector, solverResult.path.slice(Math.max(0, circleEnd - 1)), '#dc2626');
    }

    context.save();
    context.setLineDash([10, 8]);
    context.strokeStyle = '#94a3b8';
    context.lineWidth = 1.25;
    context.beginPath();
    context.moveTo(startPointOnScreen.x, startPointOnScreen.y);
    context.lineTo(startHandleOnScreen.x, startHandleOnScreen.y);
    context.moveTo(endPointOnScreen.x, endPointOnScreen.y);
    context.lineTo(endHandleOnScreen.x, endHandleOnScreen.y);
    context.stroke();
    context.restore();

    context.fillStyle = '#06b6d4';
    context.beginPath();
    context.arc(startPointOnScreen.x, startPointOnScreen.y, 7, 0, Math.PI * 2);
    context.fill();
    context.beginPath();
    context.arc(endPointOnScreen.x, endPointOnScreen.y, 7, 0, Math.PI * 2);
    context.fill();

    context.fillStyle = '#0891b2';
    context.beginPath();
    context.arc(coreStartOnScreen.x, coreStartOnScreen.y, 5, 0, Math.PI * 2);
    context.fill();
    context.beginPath();
    context.arc(coreEndOnScreen.x, coreEndOnScreen.y, 5, 0, Math.PI * 2);
    context.fill();

    context.fillStyle = '#94a3b8';
    context.beginPath();
    context.arc(startHandleOnScreen.x, startHandleOnScreen.y, 5, 0, Math.PI * 2);
    context.fill();
    context.beginPath();
    context.arc(endHandleOnScreen.x, endHandleOnScreen.y, 5, 0, Math.PI * 2);
    context.fill();

    context.fillStyle = '#111827';
    context.font = '12px "Segoe UI Variable", "Noto Sans SC", sans-serif';
    context.fillText('P0', startPointOnScreen.x + 10, startPointOnScreen.y - 10);
    context.fillText('P1', endPointOnScreen.x + 10, endPointOnScreen.y - 10);
    context.fillStyle = '#0891b2';
    context.fillText('核心起点', coreStartOnScreen.x + 10, coreStartOnScreen.y + 16);
    context.fillText('核心终点', coreEndOnScreen.x + 10, coreEndOnScreen.y + 16);
  }, [
    canvasWidth,
    endHandle,
    endPose.point,
    extendedEndPoint,
    extendedStartPoint,
    solverResult,
    startHandle,
    startPose.point,
    viewport,
  ]);

  const applyPointerUpdate = (clientX: number, clientY: number, activeTarget: DragTarget | null = dragTarget) => {
    if (!canvasRef.current || !activeTarget) {
      return;
    }

    const bounds = canvasRef.current.getBoundingClientRect();
    const projector = createProjector(viewport, Math.max(320, Math.floor(canvasWidth)), CANVAS_HEIGHT);
    const worldPoint = projector.unproject({ x: clientX - bounds.left, y: clientY - bounds.top });

    if (activeTarget === 'start-point') {
      setStartPose((current) => ({
        ...current,
        point: {
          x: clamp(worldPoint.x, WORLD_BOUNDS.minX + 20, WORLD_BOUNDS.maxX - 20),
          y: clamp(worldPoint.y, WORLD_BOUNDS.minY + 20, WORLD_BOUNDS.maxY - 20),
        },
      }));
      return;
    }

    if (activeTarget === 'end-point') {
      setEndPose((current) => ({
        ...current,
        point: {
          x: clamp(worldPoint.x, WORLD_BOUNDS.minX + 20, WORLD_BOUNDS.maxX - 20),
          y: clamp(worldPoint.y, WORLD_BOUNDS.minY + 20, WORLD_BOUNDS.maxY - 20),
        },
      }));
      return;
    }

    if (activeTarget === 'start-handle') {
      setStartPose((current) => ({
        ...current,
        heading: Math.atan2(worldPoint.y - current.point.y, worldPoint.x - current.point.x),
      }));
      return;
    }

    setEndPose((current) => ({
      ...current,
      heading: Math.atan2(worldPoint.y - current.point.y, worldPoint.x - current.point.x),
    }));
  };

  const handlePointerDown = (event: React.PointerEvent<HTMLCanvasElement>) => {
    const bounds = event.currentTarget.getBoundingClientRect();
    const pointer = { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
    const projector = createProjector(viewport, Math.max(320, Math.floor(canvasWidth)), CANVAS_HEIGHT);
    const hitCandidates: Array<{ target: DragTarget; point: Point; radius: number }> = [
      { target: 'start-handle', point: projector.project(startHandle), radius: HANDLE_HIT_RADIUS },
      { target: 'end-handle', point: projector.project(endHandle), radius: HANDLE_HIT_RADIUS },
      { target: 'start-point', point: projector.project(startPose.point), radius: POINT_HIT_RADIUS },
      { target: 'end-point', point: projector.project(endPose.point), radius: POINT_HIT_RADIUS },
    ];
    const hit = hitCandidates.find((candidate) => distance(candidate.point, pointer) <= candidate.radius);

    if (!hit) {
      return;
    }

    setDragTarget(hit.target);
    event.currentTarget.setPointerCapture(event.pointerId);
    applyPointerUpdate(event.clientX, event.clientY, hit.target);
  };

  const handlePointerMove = (event: React.PointerEvent<HTMLCanvasElement>) => {
    if (!dragTarget) {
      return;
    }

    applyPointerUpdate(event.clientX, event.clientY);
  };

  const handlePointerRelease = (event: React.PointerEvent<HTMLCanvasElement>) => {
    if (!dragTarget) {
      return;
    }

    event.currentTarget.releasePointerCapture(event.pointerId);
    setDragTarget(null);
  };

  const resetScene = () => {
    setParameterA(120);
    setStartPose({ point: { x: 160, y: 520 }, heading: -0.35 });
    setEndPose({ point: { x: 900, y: 220 }, heading: 2.64 });
  };

  const statusLabel =
    solverResult.status === 'ok'
      ? '找到最大可行 R'
      : solverResult.status === 'straight-only'
        ? '退化为纯直线'
        : solverResult.status === 'approx'
          ? '近似可行'
          : '当前条件不可用';

  return (
    <>
      <section className="control-section" aria-labelledby="straight-pad-controls-heading">
        <div className="section-heading-row">
          <h2 id="straight-pad-controls-heading">二点约束 + 首尾直线</h2>
          <p className="section-note">
            允许在 S-C-S 核心曲线两端各补一段非负直线，并在满足两点与两端切线约束的可行解里搜索最大半径 R。
          </p>
        </div>

        <div className="solver-controls-grid">
          <label className="slider-row" htmlFor="straight-pad-parameter-a">
            <span className="slider-label">A</span>
            <input
              id="straight-pad-parameter-a"
              type="range"
              min="40"
              max="220"
              step="1"
              value={parameterA}
              onChange={(event) => setParameterA(Number(event.target.value))}
            />
            <output className="slider-value">{formatValue(parameterA, 0)}</output>
          </label>

          <div className="solver-actions">
            <button type="button" className="ghost-button" onClick={resetScene}>
              重置示例
            </button>
            <p className={`solver-status solver-status-${solverResult.status === 'ok' ? 'ok' : solverResult.status === 'invalid' ? 'invalid' : 'approx'}`}>
              {statusLabel}
            </p>
          </div>
        </div>
      </section>

      <section className="metrics-section" aria-labelledby="straight-pad-metrics-heading">
        <div className="section-heading-row">
          <h2 id="straight-pad-metrics-heading">求解结果</h2>
          <p className="section-note">
            直线段长度约束为 l0 ≥ 0、l1 ≥ 0。搜索策略先扫描可行半径区间，再在最后一处可行边界附近二分收敛到最大 R。
          </p>
        </div>

        <dl className="metrics-grid solver-metrics-grid">
          <div>
            <dt>最大可行半径 R</dt>
            <dd>{formatValue(solverResult.radius)}</dd>
          </div>
          <div>
            <dt>理论下界</dt>
            <dd>{formatValue(solverResult.minRadius)}</dd>
          </div>
          <div>
            <dt>总转角 Δ</dt>
            <dd>{formatValue(angleToDegrees(solverResult.headingDelta))}°</dd>
          </div>
          <div>
            <dt>前直线长 l0</dt>
            <dd>{formatValue(solverResult.leadInLength)}</dd>
          </div>
          <div>
            <dt>后直线长 l1</dt>
            <dd>{formatValue(solverResult.leadOutLength)}</dd>
          </div>
          <div>
            <dt>缓和曲线长</dt>
            <dd>{formatValue(solverResult.transitionLength)}</dd>
          </div>
          <div>
            <dt>圆曲线长</dt>
            <dd>{formatValue(solverResult.circleLength)}</dd>
          </div>
          <div>
            <dt>总长</dt>
            <dd>{formatValue(solverResult.totalLength)}</dd>
          </div>
          <div>
            <dt>终点误差</dt>
            <dd>{formatValue(solverResult.positionError, 5)}</dd>
          </div>
        </dl>

        <div className="solver-notes">
          <div>
            <h3>允许项</h3>
            <p>核心曲线前后都允许补直线，因此目标点不必正好落在纯 S-C-S 轨迹上。</p>
          </div>
          <div>
            <h3>优化目标</h3>
            <p>在所有满足 l0 ≥ 0、l1 ≥ 0 的可行解中，优先选择半径最大的解，而不是直线最短的解。</p>
          </div>
          <div>
            <h3>图上含义</h3>
            <p>aqua 大点是给定端点，aqua 小点是核心曲线接入点，绿色段是自动补上的首尾直线。</p>
          </div>
        </div>
      </section>

      <section className="canvas-section" aria-labelledby="straight-pad-canvas-heading">
        <div className="section-heading-row">
          <h2 id="straight-pad-canvas-heading">交互画板</h2>
          <p className="section-note">
            绿色线表示自动补上的首尾直线，红线表示缓和曲线，黄线表示圆曲线，灰色柄用于定义起终点切线方向。
          </p>
        </div>

        <figure className="canvas-figure">
          <div ref={canvasHostRef} className="canvas-host">
            <canvas
              ref={canvasRef}
              aria-label="允许首尾直线的二点约束最大半径求解画板"
              onPointerDown={handlePointerDown}
              onPointerMove={handlePointerMove}
              onPointerUp={handlePointerRelease}
              onPointerCancel={handlePointerRelease}
              onPointerLeave={handlePointerRelease}
            />
          </div>
          <figcaption className="canvas-caption">
            当纯 S-C-S 无法直接命中两点时，这一版会把剩余位移分解到起终点切线方向上，只要两段补线长度都不为负，就视为可行；然后在这些可行解中继续把半径 R 向上推到可行边界。
          </figcaption>
        </figure>
      </section>
    </>
  );
}