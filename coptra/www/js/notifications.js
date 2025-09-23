// Common notification system for all pages
class NotificationSystem {
    constructor() {
        this.notification = document.getElementById('notification');
        this.init();
    }

    init() {
        if (!this.notification) {
            console.warn('Notification element not found');
            return;
        }
    }

    show(message, type = 'info', duration = 3000) {
        if (!this.notification) return;

        // Show the notification element
        this.notification.style.display = 'flex';
        
        this.notification.textContent = message;
        this.notification.className = `notification ${type} show`;
        
        // Clear any existing timeout
        if (this.timeoutId) {
            clearTimeout(this.timeoutId);
        }
        
        // Auto-hide after duration
        this.timeoutId = setTimeout(() => {
            this.hide();
        }, duration);
    }

    hide() {
        if (!this.notification) return;
        
        this.notification.classList.remove('show');
        
        // Wait for animation to complete before hiding
        setTimeout(() => {
            this.notification.style.display = 'none';
        }, 400);
        
        // Clear timeout if exists
        if (this.timeoutId) {
            clearTimeout(this.timeoutId);
            this.timeoutId = null;
        }
    }

    success(message, duration = 3000) {
        this.show(message, 'success', duration);
    }

    error(message, duration = 5000) {
        this.show(message, 'error', duration);
    }

    info(message, duration = 3000) {
        this.show(message, 'info', duration);
    }

    warning(message, duration = 4000) {
        this.show(message, 'warning', duration);
    }
}

// Global notification instance
window.notifications = new NotificationSystem();

// Export for module usage
if (typeof module !== 'undefined' && module.exports) {
    module.exports = NotificationSystem;
}
