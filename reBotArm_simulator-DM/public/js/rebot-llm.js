(function () {
  const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
  const STATUS_MAP = {
    notStarted: { key: 'llm.notStarted', className: 'mini-pill offline' },
    starting: { key: 'llm.starting', className: 'mini-pill warn' },
    started: { key: 'llm.started', className: 'mini-pill online' },
    startFail: { key: 'llm.startFail', className: 'mini-pill error' }
  };

  class ReBotLLMUI {
    constructor() {
      this.started = false;
      this.statusCode = 'notStarted';
      this._lastMsg = null;
      this.config = { textAgentUrl: '', mcpUrl: '' };
      this.elements = {};
    }

    init() {
      this.elements = {
        status: document.getElementById('llm-status'),
        chatMessages: document.getElementById('llm-chat-messages'),
        input: document.getElementById('llm-input'),
        sendBtn: document.getElementById('llm-send'),
        startBtn: document.getElementById('llm-start'),
        stopBtn: document.getElementById('llm-stop'),
        message: document.getElementById('llm-message')
      };

      if (!this.elements.status) {
        console.error('LLM UI elements not found');
        return;
      }

      this.elements.startBtn.addEventListener('click', () => this.handleStart());
      this.elements.stopBtn.addEventListener('click', () => this.handleStop());
      this.elements.sendBtn.addEventListener('click', () => this.handleSend());
      this.elements.input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
          e.preventDefault();
          this.handleSend();
        }
      });

      // Initial config fetch (display only)
      fetch('/api/mcp/config').then(r => r.json()).then(cfg => {
        this.config = cfg;
        this.setMessage('llm.backendInfo', { textAgent: cfg.textAgentUrl, mcp: cfg.mcpUrl });
      }).catch(() => {
        this.setMessage('msg.llmLoadCfgFail');
      });

      this.updateStatus('notStarted');

      if (window.rebotI18n) {
        window.rebotI18n.onLangChange(() => this._rerender());
      }
    }

    _rerender() {
      this.updateStatus(this.statusCode);
      if (this._lastMsg) {
        this.elements.message.textContent = t(this._lastMsg.key, this._lastMsg.params);
      }
    }

    setMessage(key, params) {
      this._lastMsg = { key, params };
      this.elements.message.textContent = t(key, params);
    }

    updateStatus(code) {
      this.statusCode = code;
      const map = STATUS_MAP[code] || STATUS_MAP.notStarted;
      this.elements.status.textContent = t(map.key);
      this.elements.status.className = map.className;
    }

    async handleStart() {
      console.log('[LLM UI] handleStart');
      this.elements.startBtn.disabled = true;
      this.updateStatus('starting');
      this.setMessage('msg.llmConnecting');
      this.addMessage('system', t('llm.connectingVm'));

      try {
        // Health check
        const res = await fetch('/api/llm/health');
        const data = await res.json().catch(() => ({}));
        if (!res.ok || data.ok === false) {
          throw new Error(data.error || `text-agent HTTP ${res.status}`);
        }

        this.started = true;
        this.updateStatus('started');
        this.setMessage('msg.llmConnected');
        this.elements.stopBtn.disabled = false;
        this.elements.input.disabled = false;
        this.elements.sendBtn.disabled = false;
        this.addMessage('system', t('llm.welcome'));
        this.elements.input.focus();
      } catch (e) {
        console.error('[LLM UI] start failed:', e);
        this.updateStatus('startFail');
        this.setMessage('msg.llmConnectFail', { err: e.message });
        this.addMessage('error', t('llm.connectFailDetail', { err: e.message }));
        this.elements.startBtn.disabled = false;
      }
    }

    handleStop() {
      this.started = false;
      this.updateStatus('notStarted');
      this.setMessage('msg.llmStopped');
      this.elements.stopBtn.disabled = true;
      this.elements.input.disabled = true;
      this.elements.sendBtn.disabled = true;
      this.addMessage('system', t('llm.stoppedChat'));
      this.elements.startBtn.disabled = false;

      // Notify backend to clear context (if reset endpoint exists)
      fetch('/api/llm/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: '__reset__', reset: true })
      }).catch(() => {});
    }

    async handleSend() {
      const text = this.elements.input.value.trim();
      if (!text) return;
      if (!this.started) {
        this.addMessage('error', t('msg.llmStartPrompt'));
        return;
      }

      this.elements.input.value = '';
      this.addMessage('user', text);
      this.addMessage('loading', t('llm.thinking'));
      this.elements.sendBtn.disabled = true;

      try {
        const res = await fetch('/api/llm/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ text })
        });

        this.removeLoadingMessage();

        if (!res.ok) {
          const errText = await res.text();
          this.addMessage('error', `HTTP ${res.status}: ${errText.substring(0, 200)}`);
          return;
        }

        const data = await res.json();
        if (data.ok === false) {
          this.addMessage('error', t('llm.errorMsg', { err: data.error || t('llm.unknownError') }));
          return;
        }

        // Show reply
        if (data.text) {
          this.addMessage('assistant', data.text);
        } else {
          this.addMessage('assistant', t('llm.noReply'));
        }

        // Show tool call traces
        const events = data.events || [];
        for (const evt of events) {
          if (evt.type === 'tool') {
            this.addMessage('tool', t('llm.toolEvent', {
              name: evt.name,
              args: JSON.stringify(evt.arguments),
              result: JSON.stringify(evt.result).substring(0, 200)
            }));
          } else if (evt.type === 'info') {
            this.addMessage('info', evt.message);
          } else if (evt.type === 'error') {
            this.addMessage('error', evt.message);
          }
        }
      } catch (e) {
        this.removeLoadingMessage();
        this.addMessage('error', t('llm.requestFail', { err: e.message }));
      } finally {
        this.elements.sendBtn.disabled = false;
        this.elements.input.focus();
      }
    }

    addMessage(role, content) {
      const div = document.createElement('div');
      div.className = `llm-message llm-message-${role}`;
      if (content && content.includes('\n')) {
        div.style.whiteSpace = 'pre-wrap';
      }
      div.textContent = content;
      this.elements.chatMessages.appendChild(div);
      this.elements.chatMessages.scrollTop = this.elements.chatMessages.scrollHeight;
    }

    removeLoadingMessage() {
      const loading = this.elements.chatMessages.querySelector('.llm-message-loading');
      if (loading) loading.remove();
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      window.llmUI = new ReBotLLMUI();
      window.llmUI.init();
    });
  } else {
    window.llmUI = new ReBotLLMUI();
    window.llmUI.init();
  }
})();
