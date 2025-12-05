/*
  auth.ts
  Frontend-only authentication using localStorage.
  Falls back to a local demo implementation (no server SDK).

  Exports:
  - authClient: object with signIn, signUp, signOut, getSession, setSession
  - signIn, signUp, signOut helpers
  - useSession hook for React components
*/

import React from 'react';

type User = { id: string; email: string; name?: string };
type Session = { user: User; token: string } | null;

class LocalAuthClient {
  public storageKey = 'better-auth-session';

  async signIn(email: string, password: string) {
    await new Promise((r) => setTimeout(r, 400));
    if (!email || !password) throw new Error('Email and password are required');
    const user: User = { id: Date.now().toString(), email, name: email.split('@')[0] };
    const session: Session = { user, token: `demo-token-${Date.now()}` };
    localStorage.setItem(this.storageKey, JSON.stringify(session));
    return { data: session };
  }

  async signUp(email: string, password: string, name?: string) {
    await new Promise((r) => setTimeout(r, 400));
    if (!email || !password) throw new Error('Email and password are required');
    const user: User = { id: Date.now().toString(), email, name: name || email.split('@')[0] };
    const session: Session = { user, token: `demo-token-${Date.now()}` };
    localStorage.setItem(this.storageKey, JSON.stringify(session));
    return { data: session };
  }

  async signOut() {
    localStorage.removeItem(this.storageKey);
  }

  getSession(): Session {
    const raw = localStorage.getItem(this.storageKey);
    return raw ? JSON.parse(raw) : null;
  }

  setSession(session: Session) {
    if (session) localStorage.setItem(this.storageKey, JSON.stringify(session));
    else localStorage.removeItem(this.storageKey);
  }
}

// Use local auth client (no server SDK needed for demo)
const authClient = new LocalAuthClient();

export { authClient };
export const signIn = (email: string, password: string) => authClient.signIn(email, password);
export const signUp = (email: string, password: string, name?: string) => authClient.signUp(email, password, name);
export const signOut = () => authClient.signOut();

export function useSession() {
  const [session, setSession] = React.useState<Session>(authClient.getSession());
  const [isPending, setIsPending] = React.useState(false);

  React.useEffect(() => {
    setIsPending(true);
    const stored = authClient.getSession();
    setSession(stored);
    setIsPending(false);

    const handleStorage = (e: StorageEvent) => {
      if (e.key === authClient.storageKey) {
        setSession(e.newValue ? JSON.parse(e.newValue) : null);
      }
    };
    window.addEventListener('storage', handleStorage);
    return () => window.removeEventListener('storage', handleStorage);
  }, []);

  return { data: session, isPending };
}

export default authClient;
