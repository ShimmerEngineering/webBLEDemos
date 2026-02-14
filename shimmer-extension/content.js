function cleanTitle(t) {
  if (!t) return "";
  return t
    .replace(/\s*\|\s*Netflix\s*$/i, "")
    .replace(/\s*-\s*Netflix\s*$/i, "")
    .replace(/^Netflix\s*-\s*/i, "")
    .trim();
}

function getNetflixTitle() {
  // Player overlay title (most important for /watch)
  const t1 = document.querySelector('[data-uia="video-title"]')?.textContent?.trim();
  // Often episode name / extra context
  const sub1 = document.querySelector('[data-uia="video-subtitle"]')?.textContent?.trim();
  const sub2 = document.querySelector('[data-uia="video-title"] [data-uia="video-subtitle"]')?.textContent?.trim();

  const title = cleanTitle(t1);
  const subtitle = cleanTitle(sub1 || sub2);

  if (title && subtitle) return `${title} — ${subtitle}`;
  if (title) return title;

  // Non-player pages (details pages etc.)
  const t2 = document.querySelector('[data-uia="title-info-title"]')?.textContent?.trim();
  if (t2) return cleanTitle(t2);

  // Meta fallback (sometimes present, sometimes not on /watch)
  const og = document.querySelector('meta[property="og:title"]')?.content?.trim();
  if (og && cleanTitle(og).toLowerCase() !== "netflix") return cleanTitle(og);

  // Last resort
  return cleanTitle(document.title) || "Unknown";
}

function getYouTubeTitle() {
  const yt1 = document.querySelector('ytd-watch-metadata h1 yt-formatted-string')?.textContent?.trim();
  if (yt1) return yt1;

  const yt2 = document.querySelector('#title h1 yt-formatted-string')?.textContent?.trim();
  if (yt2) return yt2;

  const yt3 = document.querySelector('h1.title yt-formatted-string')?.textContent?.trim();
  if (yt3) return yt3;

  const og = document.querySelector('meta[property="og:title"]')?.content?.trim();
  if (og) return og;

  return cleanTitle(document.title) || "Unknown";
}

function getTitle() {
  const host = location.hostname;
  if (host.includes("netflix.com")) return getNetflixTitle();
  if (host.includes("youtube.com")) return getYouTubeTitle();

  const og = document.querySelector('meta[property="og:title"]')?.content?.trim();
  return cleanTitle(og || document.title) || "Unknown";
}

function broadcastData() {
	
  const video = document.querySelector('video');
  const isAdShowing = document.querySelector('.ad-showing, .ad-interrupting') !== null;

  if (video) {
    chrome.runtime.sendMessage({
      type: "VIDEO_UPDATE",
      time: video.currentTime,
      title: getTitle(),
      isPaused: video.paused,
      isAd: isAdShowing
    });
  }
}


setInterval(broadcastData, 200);

const observer = new MutationObserver(broadcastData);
observer.observe(document.documentElement, { childList: true, subtree: true });
