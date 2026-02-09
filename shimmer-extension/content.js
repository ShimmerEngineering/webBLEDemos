function broadcastData() {
    const video = document.querySelector('video');
    const titleElement = document.querySelector('#title h1 yt-formatted-string, ytd-video-primary-info-renderer h1.title');
    const isAdShowing = document.querySelector('.ad-showing, .ad-interrupting') !== null;
    
    if (video) {
        chrome.runtime.sendMessage({
            type: "VIDEO_UPDATE",
            time: video.currentTime,
            title: titleElement ? titleElement.textContent.trim() : document.title,
            isPaused: video.paused,
            isAd: isAdShowing
        });
    }
}

setInterval(broadcastData, 200);

const observer = new MutationObserver(() => {
    broadcastData();
});

observer.observe(document.body, { childList: true, subtree: true });