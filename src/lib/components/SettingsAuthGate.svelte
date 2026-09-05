<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<script lang="ts">
  // Password gate shown before the Settings panel opens (see stores/auth.ts, stores/settings.ts
  // `security`). Same show()-returns-a-promise convention as ConfirmDialog.svelte, and the same
  // backdrop/box/Escape-to-cancel structure — but with real password fields instead of a checkbox.
  import { t } from 'svelte-i18n';
  import { settings } from '$lib/stores/settings';
  import { auth } from '$lib/stores/auth';
  import { generateSalt, hashPassword } from '$lib/helpers/passwordHash';

  type Mode = 'setup' | 'unlock';

  let open = $state(false);
  let mode = $state<Mode>('unlock');
  let password = $state('');
  let confirmPassword = $state('');
  let error = $state('');
  let resolver: ((value: boolean) => void) | null = null;
  let passwordInput = $state<HTMLInputElement>();

  /** Show the gate and resolve true once unlocked, false if cancelled/backdrop/Escape. */
  export function show(requestedMode: Mode): Promise<boolean> {
    mode = requestedMode;
    password = '';
    confirmPassword = '';
    error = '';
    open = true;
    queueMicrotask(() => passwordInput?.focus());
    return new Promise<boolean>((resolve) => {
      resolver = resolve;
    });
  }

  function close(value: boolean) {
    open = false;
    if (resolver) {
      resolver(value);
      resolver = null;
    }
  }

  async function submit() {
    if (!password) {
      error = $t('security.emptyError');
      return;
    }
    if (mode === 'setup') {
      if (password !== confirmPassword) {
        error = $t('security.mismatchError');
        return;
      }
      const salt = generateSalt();
      const hash = await hashPassword(password, salt);
      settings.patch({ security: { passwordHash: hash, passwordSalt: salt } });
      auth.unlock();
      close(true);
      return;
    }
    // unlock
    const { passwordHash, passwordSalt } = $settings.security;
    const attempt = passwordSalt ? await hashPassword(password, passwordSalt) : null;
    if (attempt && attempt === passwordHash) {
      auth.unlock();
      close(true);
    } else {
      error = $t('security.wrongPasswordError');
      password = '';
      passwordInput?.focus();
    }
  }

  function handleKeydown(e: KeyboardEvent) {
    if (e.key === 'Escape') close(false);
    if (e.key === 'Enter') submit();
  }
</script>

{#if open}
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <div class="dialog-backdrop" onclick={() => close(false)} onkeydown={handleKeydown}>
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <!-- svelte-ignore a11y_no_static_element_interactions -->
    <div class="dialog-box" onclick={(e) => e.stopPropagation()}>
      <div class="dialog-title">{mode === 'setup' ? $t('security.setupTitle') : $t('security.unlockTitle')}</div>
      <div class="dialog-message">{mode === 'setup' ? $t('security.setupMessage') : $t('security.unlockMessage')}</div>

      <div class="auth-fields">
        <input
          type="password"
          class="auth-input"
          bind:this={passwordInput}
          bind:value={password}
          placeholder={$t('security.passwordPlaceholder')}
          onkeydown={handleKeydown}
        />
        {#if mode === 'setup'}
          <input
            type="password"
            class="auth-input"
            bind:value={confirmPassword}
            placeholder={$t('security.confirmPlaceholder')}
            onkeydown={handleKeydown}
          />
        {/if}
      </div>

      {#if error}
        <div class="auth-error">{error}</div>
      {/if}

      <div class="dialog-buttons">
        <button class="dialog-btn dialog-btn-cancel" onclick={() => close(false)}>{$t('dialog.cancel')}</button>
        <button class="dialog-btn dialog-btn-primary" onclick={submit}>
          {mode === 'setup' ? $t('security.setButton') : $t('security.unlockButton')}
        </button>
      </div>
    </div>
  </div>
{/if}

<style>
  .dialog-backdrop {
    position: fixed;
    inset: 0;
    z-index: 9999;
    background: rgba(0, 0, 0, 0.55);
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .dialog-box {
    background: #2e2e2e;
    border: 1px solid var(--mx-red-dim, rgba(224, 48, 44, 0.45));
    border-radius: 8px;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.5);
    padding: 20px 24px 16px;
    min-width: 340px;
    max-width: 480px;
  }

  .dialog-title {
    font-size: 14px;
    font-weight: 700;
    color: #e0e0e0;
    margin-bottom: 10px;
  }

  .dialog-message {
    font-size: 12px;
    color: #bbb;
    line-height: 1.5;
    white-space: pre-line;
    margin-bottom: 16px;
  }

  .auth-fields {
    display: flex;
    flex-direction: column;
    gap: 8px;
    margin-bottom: 8px;
  }

  .auth-input {
    height: 32px;
    box-sizing: border-box;
    padding: 0 10px;
    background: #434343;
    border: 1px solid #555;
    border-radius: 4px;
    color: #e0e0e0;
    font-size: 13px;
  }

  .auth-input:focus {
    outline: none;
    border-color: var(--mx-red, #e0302c);
  }

  .auth-error {
    font-size: 12px;
    color: #ff6b5b;
    margin-bottom: 8px;
  }

  .dialog-buttons {
    display: flex;
    justify-content: flex-end;
    gap: 8px;
    flex-wrap: wrap;
  }

  .dialog-btn {
    padding: 6px 14px;
    font-size: 12px;
    font-weight: 600;
    border-radius: 4px;
    border: 1px solid #555;
    background: #434343;
    color: #e0e0e0;
    cursor: pointer;
    transition: background 0.15s;
  }

  .dialog-btn:hover {
    background: #505050;
  }

  .dialog-btn-cancel {
    color: #999;
  }

  .dialog-btn-primary {
    background: var(--mx-red, #e0302c);
    border-color: var(--mx-red, #e0302c);
    color: #fff;
  }

  .dialog-btn-primary:hover {
    background: var(--mx-red-bright, #ff4d47);
    border-color: var(--mx-red-bright, #ff4d47);
  }
</style>
