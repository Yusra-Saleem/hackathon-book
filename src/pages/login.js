import React from 'react';
import Layout from '@theme/Layout';

function Login() {
  const handleSubmit = async (event) => {
    event.preventDefault();
    const formData = new FormData(event.target);
    const data = Object.fromEntries(formData.entries());

    const response = await fetch('/api/auth/token', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded',
      },
      body: new URLSearchParams({
        username: data.email,
        password: data.password,
      }),
    });

    if (response.ok) {
      const { access_token } = await response.json();
      localStorage.setItem('token', access_token);
      window.location.href = '/';
    } else {
      alert('Login failed');
    }
  };

  return (
    <Layout title="Login">
      <div
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '50vh',
          flexDirection: 'column',
        }}
      >
        <h1>Login</h1>
        <form onSubmit={handleSubmit}>
          <div style={{ marginBottom: '1rem' }}>
            <label>Email</label>
            <input type="email" name="email" style={{ marginLeft: '0.5rem' }} />
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <label>Password</label>
            <input type="password" name="password" style={{ marginLeft: '0.5rem' }} />
          </div>
          <button type="submit">Login</button>
        </form>
      </div>
    </Layout>
  );
}

export default Login;