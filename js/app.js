import { initCamera, getVideo } from './camera.js';
import { initTracking, processTrackingFrame } from './tracking.js?v=tracking-fix-5';
import { drawTracking } from './visualization.js?v=tracking-loop-fix-1';
import {
  initUI,
  updateHUD,
  isDebugDrawEnabled,
  isHomographyDebugEnabled,
  consumeRedetectRequest
} from './ui.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let loopStarted = false;
const trackingData = {
  loading: true,
  trackedCount: 0,
  trackedPoints: [],
  prevPoints: [],
  status: 'Tracking LOST',
  homography: null,
  pose: null,
  poseEssential: null,
  poseDebug: null
};

function startRenderLoop() {
  if (loopStarted) {
    return;
  }
  loopStarted = true;
  processFrame();
}

function processFrame() {
  const video = getVideo();
  if (!video) {
    requestAnimationFrame(processFrame);
    return;
  }

  // Safety: never process until camera metadata is available.
  if (video.videoWidth === 0 || video.videoHeight === 0) {
    requestAnimationFrame(processFrame);
    return;
  }

  if (video.readyState >= video.HAVE_CURRENT_DATA) {
    const vw = video.videoWidth;
    const vh = video.videoHeight;

    if (canvas.width !== vw || canvas.height !== vh) {
      canvas.width = vw;
      canvas.height = vh;
    }

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

    // Tracking updates continuously in the frame loop; button only forces a reset.
    const frameTrackingData = processTrackingFrame(canvas, {
      forceRedetect: consumeRedetectRequest()
    });
    Object.assign(trackingData, frameTrackingData);

    drawTracking(
      ctx,
      trackingData,
      video,
      isDebugDrawEnabled(),
      isHomographyDebugEnabled()
    );

    updateHUD(trackingData.trackedCount, trackingData.status);
  }

  requestAnimationFrame(processFrame);
}

async function initApp() {
  initUI();
  await initCamera();
  await initTracking(getVideo());

  const video = getVideo();
  if (!video) {
    return;
  }

  // Mobile browsers can fire play before listeners are attached, so we use
  // multiple readiness hooks and an immediate readyState check.
  video.addEventListener('play', startRenderLoop);
  video.addEventListener('playing', startRenderLoop);
  video.addEventListener('loadeddata', startRenderLoop);
  video.addEventListener('loadedmetadata', startRenderLoop);

  if (video.readyState >= video.HAVE_CURRENT_DATA) {
    startRenderLoop();
  }
}

initApp().catch((error) => {
  console.error('App initialization failed:', error);
});
